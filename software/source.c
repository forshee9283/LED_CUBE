
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/sem.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "ws2812.pio.h"
#include "prox.pio.h"

#define BRIGHTNESS 0xffff
#define WS2812_PIN_BASE 2
#define NUM_PIXELS 19

#define PROX_TOP_PIO pio0 //PIO block for proximity sensor
#define PROX_TOP_SM 0 //state machine number for proximity sensor
#define PROX_TOP_PIN 7 //GPIO number for the proximity sensor
#define WS2812_PIO pio1
#define WS2812_SM 0
#define TIMER_IRQ 0

#define INTERRUPT_FREQUENCY 60
#define TIMER_PERIOD_US (1000000 / INTERRUPT_FREQUENCY)

volatile int prox_top = 0;

static uint8_t *current_strip_out;
static inline void put_pixel(uint32_t pixel_grb) {
    *current_strip_out++ = pixel_grb & 0xffu;
    *current_strip_out++ = (pixel_grb >> 8u) & 0xffu;
    *current_strip_out++ = (pixel_grb >> 16u) & 0xffu;
}

int prox_setup(PIO pio_prox, int start_pin, int sm, const float clk_div){
    uint offset_prox = pio_add_program(pio_prox, &prox_program);
    pio_sm_claim(pio_prox, sm);//Panic if unavailible
    prox_init(pio_prox, sm, offset_prox, start_pin, clk_div);
    pio_sm_set_enabled(pio_prox, sm, true);
}

bool timer_callback (struct repeating_timer *t) {
    printf("touch_state: %8u \n", (prox_top));
    prox_top = 0;
    return true;
}

//Set up strips
typedef struct {
    uint8_t *data;
    uint data_len;
    uint frac_brightness; // 256 = *1.0;
} strip_t;

static uint8_t strip0_data[NUM_PIXELS * 3];
static uint8_t strip1_data[NUM_PIXELS * 3];
static uint8_t strip2_data[NUM_PIXELS * 3];
static uint8_t strip3_data[NUM_PIXELS * 3];

//Back and top
strip_t strip0 = { 
        .data = strip0_data,
        .data_len = sizeof(strip0_data),
        .frac_brightness = 0x40,
};
//Left side
strip_t strip1 = {
        .data = strip1_data,
        .data_len = sizeof(strip1_data),
        .frac_brightness = 0x100,
};
//Front
strip_t strip2 = {
        .data = strip1_data,
        .data_len = sizeof(strip2_data),
        .frac_brightness = 0x100,
};//Right side
strip_t strip3 = {
        .data = strip1_data,
        .data_len = sizeof(strip3_data),
        .frac_brightness = 0x100,
};

strip_t *strips[] = {
        &strip0,
        &strip1,
        &strip2,
        &strip3,
};

#define VALUE_PLANE_COUNT (8 + FRAC_BITS)
// we store value (8 bits + fractional bits of a single color (R/G/B/W) value) for multiple
// strips of pixels, in bit planes. bit plane N has the Nth bit of each strip of pixels.
typedef struct {
    // stored MSB first
    uint32_t planes[VALUE_PLANE_COUNT];
} value_bits_t;

#define DMA_CHANNEL 0
#define DMA_CB_CHANNEL 1

#define DMA_CHANNEL_MASK (1u << DMA_CHANNEL)
#define DMA_CB_CHANNEL_MASK (1u << DMA_CB_CHANNEL)
#define DMA_CHANNELS_MASK (DMA_CHANNEL_MASK | DMA_CB_CHANNEL_MASK)

// start of each value fragment (+1 for NULL terminator)
static uintptr_t fragment_start[NUM_PIXELS * 4 + 1];

// posted when it is safe to output a new set of values
static struct semaphore reset_delay_complete_sem;
// alarm handle for handling delay
alarm_id_t reset_delay_alarm_id;

int64_t reset_delay_complete(alarm_id_t id, void *user_data) {
    reset_delay_alarm_id = 0;
    sem_release(&reset_delay_complete_sem);
    // no repeat
    return 0;
}

void __isr dma_complete_handler() {
    if (dma_hw->ints0 & DMA_CHANNEL_MASK) {
        // clear IRQ
        dma_hw->ints0 = DMA_CHANNEL_MASK;
        // when the dma is complete we start the reset delay timer
        if (reset_delay_alarm_id) cancel_alarm(reset_delay_alarm_id);
        reset_delay_alarm_id = add_alarm_in_us(400, reset_delay_complete, NULL, true);
    }
}

void dma_init(PIO WS2812_PIO, uint WS2812_SM) {
    dma_claim_mask(DMA_CHANNELS_MASK);

    // main DMA channel outputs 8 word fragments, and then chains back to the chain channel
    dma_channel_config channel_config = dma_channel_get_default_config(DMA_CHANNEL);
    channel_config_set_dreq(&channel_config, pio_get_dreq(WS2812_PIO, WS2812_SM, true));
    channel_config_set_chain_to(&channel_config, DMA_CB_CHANNEL);
    channel_config_set_irq_quiet(&channel_config, true);
    dma_channel_configure(DMA_CHANNEL,
                          &channel_config,
                          &WS2812_PIO->txf[WS2812_SM],
                          NULL, // set by chain
                          8, // 8 words for 8 bit planes
                          false);

    // chain channel sends single word pointer to start of fragment each time
    dma_channel_config chain_config = dma_channel_get_default_config(DMA_CB_CHANNEL);
    dma_channel_configure(DMA_CB_CHANNEL,
                          &chain_config,
                          &dma_channel_hw_addr(
                                  DMA_CHANNEL)->al3_read_addr_trig,  // ch DMA config (target "ring" buffer size 4) - this is (read_addr trigger)
                          NULL, // set later
                          1,
                          false);

    irq_set_exclusive_handler(DMA_IRQ_0, dma_complete_handler);
    dma_channel_set_irq0_enabled(DMA_CHANNEL, true);
    irq_set_enabled(DMA_IRQ_0, true);
}

void output_strips_dma(value_bits_t *bits, uint value_length) {
    for (uint i = 0; i < value_length; i++) {
        fragment_start[i] = (uintptr_t) bits[i].planes; // MSB first
    }
    fragment_start[value_length] = 0;
    dma_channel_hw_addr(DMA_CB_CHANNEL)->al3_read_addr_trig = (uintptr_t) fragment_start;
}

int main(){
    stdio_init_all();
    static const float pio_clk_div = 1; //This should be tuned for the size of the sensors keep as low as possible
    prox_setup(PROX_TOP_PIO, PROX_TOP_PIN, PROX_TOP_SM, pio_clk_div);

    // Initialize hardware timer
    struct repeating_timer timer;

    // Initialize the timer with the given period and enable interrupts
    add_repeating_timer_ms(-1000, timer_callback, NULL, &timer); 

    //
    uint ws2812_offset = pio_add_program(WS2812_PIO, &ws2812_parallel_program);
    ws2812_parallel_program_init(WS2812_PIO, WS2812_SM, ws2812_offset, WS2812_PIN_BASE, count_of(strips), 800000);
    
    int prox_temp =0;

    while (true){
        prox_temp =pio_sm_get(PROX_TOP_PIO,0);
            if (prox_temp>prox_top){
                prox_top = prox_temp;
            }
    //printf("touch_state: %8u \n", (prox_top));
    //printf("touch_state: %32b \n", pio_sm_get(PROX_PIO,0));
    }
    return 0;
}