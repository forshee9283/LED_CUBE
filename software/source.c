
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/sem.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
//#include "ws2812.pio.h"
#include "prox.pio.h"

#define BRIGHTNESS 0xffff
#define WS2812_PIN_BASE 2

#define PROX_TOP_PIO pio0 //PIO block for proximity sensor
#define PROX_TOP_SM 0 //state machine number for proximity sensor
#define PROX_TOP_PIN 7 //GPIO number for the proximity sensor

#define TIMER_IRQ 0

#define INTERRUPT_FREQUENCY 60
#define TIMER_PERIOD_US (1000000 / INTERRUPT_FREQUENCY)

volatile int prox_top = 0;

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

int main(){
    stdio_init_all();
    static const float pio_clk_div = 1; //This should be tuned for the size of the sensors keep as low as possible
    prox_setup(PROX_TOP_PIO, PROX_TOP_PIN, PROX_TOP_SM, pio_clk_div);

    // Initialize hardware timer
    struct repeating_timer timer;

    // Initialize the timer with the given period and enable interrupts
    add_repeating_timer_ms(-1000, timer_callback, NULL, &timer); 

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