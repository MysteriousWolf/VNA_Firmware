//
// Created by matej on 02/09/2024.
//

#include "../Inc/vna_core.h"

uint8_t main_thread_stack[MAIN_THREAD_STACK_SIZE];
TX_THREAD main_thread_ptr;

uint32_t state = 0;

void vna_init() {
    // Create the main thread
    tx_thread_create(&main_thread_ptr, (char*)"Main VNA Thread", vna_thread_entry, 0, main_thread_stack, THREAD_STACK_SIZE, 1, 1, 1, TX_AUTO_START);
}

[[noreturn]] VOID vna_thread_entry(ULONG initial_input) {
    /* Initialize the VNA */

    // Turn on all the LEDs to indicate that the VNA is starting up
    set_led_state(0, LED_ON);
    set_led_state(1, LED_ON);
    set_led_state(2, LED_ON);

    // Start the timers
    init_utils();

    /* Initialize the VCO */
    // Set BUSY animation to LED 1 to indicate that the VCO is starting up
    set_led_state(0, LED_BUSY);

    // Wait for the LDOs to stabilize (10 ticks is around 100 ms)
    tx_thread_sleep(10);

    // Initialize the VCO
    init_STUW81300(false);

    // Wait for the PLL to lock
    tx_thread_sleep(100);
    /*while (!check_lock_STUW81300()) {
        state = read_STUW81300(STUW81300_REG0);
        state = read_STUW81300(STUW81300_REG1);
        state = read_STUW81300(STUW81300_REG2);
        state = read_STUW81300(STUW81300_REG3);
        state = read_STUW81300(STUW81300_REG4);
        state = read_STUW81300(STUW81300_REG5);
        state = read_STUW81300(STUW81300_REG6);
        state = read_STUW81300(STUW81300_REG7);
        state = read_STUW81300(STUW81300_REG8);
        state = read_STUW81300(STUW81300_REG9);
        state = read_STUW81300(STUW81300_REG10);
        state = read_STUW81300(STUW81300_REG11);
        //send_STUW81300(STUW81300_REG11, 123456789);
        tx_thread_sleep(10); // Wait for the PLL to lock
    }*/

    // Turn off the busy LED 1 to indicate that the VCO is ready
    set_led_state(0, LED_OFF);


    /* Initialize the ADC */

    // Set BUSY animation to LED 2 to indicate that the ADC is starting up
    set_led_state(1, LED_BUSY);

    // Initialize the ADC
    init_ADS4222();

    // to read ADS4222 registers and make sure the ADC is working
    /*while (true) {
        state = read_ADS4222(ADS4222_REG03);
        state = read_ADS4222(ADS4222_REG29);
        state = read_ADS4222(ADS4222_REG3D);
        state = read_ADS4222(ADS4222_REG41);
        tx_thread_sleep(10); // Wait for the PLL to lock
    }*/

    tx_thread_sleep(100);

    // Turn off the busy LED 2 to indicate that the ADC is ready
    set_led_state(1, LED_OFF);


    /* Initialize the FPGA Phase Detector */

    // Set BUSY animation to LED 3 to indicate that the FPGA is starting up
    set_led_state(2, LED_BUSY);

    // Initialize the FPGA
    //init_ICE40UP5K_PD();

    tx_thread_sleep(100);

    // Turn off the busy LED 3 to indicate that the ADC is ready
    set_led_state(2, LED_OFF);

    // Busy wait for now
    while (true)
        tx_thread_sleep(10);
}
