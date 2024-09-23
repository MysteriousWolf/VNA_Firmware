//
// Created by matej on 02/09/2024.
//

#include "../Inc/vna_core.h"

vna_status status = VNA_STATUS_UNINITIALIZED;

TX_EVENT_FLAGS_GROUP measurement_event_flags;

uint8_t main_thread_stack[MAIN_THREAD_STACK_SIZE];
TX_THREAD main_thread_ptr;

uint8_t main_queue_stack[MAIN_QUEUE_STACK_SIZE];
TX_QUEUE main_queue;

uint32_t state = 0;

void vna_init() {
    // Set status to initializing signaling
    status = VNA_STATUS_INIT_SIG;

    // Initialize the signaling (LEDs, buttons, etc.)
    vna_signaling_init();

    // Set status to initializing calibration data
    status = VNA_STATUS_INIT_CALIB;

    // Initialize the calibration data
    vna_meas_init();

    // Set the status to initializing measurement data
    status = VNA_STATUS_INIT_MEAS;

    // Initialize the measurement data
    vna_meas_init();

    // Set status to initializing
    status = VNA_STATUS_INIT;

    // Create event flags
    tx_event_flags_create(&measurement_event_flags, (char*)"Measurement Event Flags");

    // Create the main task queue
    tx_queue_create(&main_queue, (char*)"Main Task Queue", 1, main_queue_stack, MAIN_QUEUE_STACK_SIZE);

    // Create the main thread
    tx_thread_create(&main_thread_ptr, (char*)"Main VNA Thread", vna_thread_entry, 0, main_thread_stack,
                     MAIN_THREAD_STACK_SIZE, 1, 1, 1, TX_AUTO_START);

    // Set status to initializing command engine
    status = VNA_STATUS_INIT_CMD;

    // Initialize the command engine
    vna_init_cmd_engine();
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
    status = VNA_STATUS_INIT_VCO;

    // Set BUSY animation to LED 1 to indicate that the VCO is starting up
    set_led_state(0, LED_BUSY);

    // Wait for the LDOs to stabilize (10 ticks is around 100 ms)
    tx_thread_sleep(10);

    // Initialize the VCO
    init_STUW81300(false);

#ifdef TEST_MODE
    // Wait a bit
    tx_thread_sleep(100);
    // Read STUW81300 registers and make sure the VCO is working
    for (int i = 0; i < 20; i++) {
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
    }
#endif

#ifndef TEST_MODE
    // TODO uncomment when testing the entire setup!
    // Wait for the PLL to lock
    /*while (!check_lock_STUW81300()) {
        tx_thread_sleep(10); // Wait for the PLL to lock
    }*/
#endif

    // Turn off the busy LED 1 to indicate that the VCO is ready
    set_led_state(0, LED_OFF);


    /* Initialize the ADC */
    status = VNA_STATUS_INIT_ADC;

    // Set BUSY animation to LED 2 to indicate that the ADC is starting up
    set_led_state(1, LED_BUSY);

    // Initialize the ADC
    init_ADS4222();

#ifdef TEST_MODE
    // Wait a bit
    tx_thread_sleep(100);

    // Read ADS4222 registers and make sure the ADC is working
    for (int i = 0; i < 20; i++) {
        state = read_ADS4222(ADS4222_REG03);
        state = read_ADS4222(ADS4222_REG29);
        state = read_ADS4222(ADS4222_REG3D);
        state = read_ADS4222(ADS4222_REG41);
        tx_thread_sleep(10); // Wait for the PLL to lock
    }
#endif

    // Turn off the busy LED 2 to indicate that the ADC is ready
    set_led_state(1, LED_OFF);


    /* Initialize the FPGA Phase Detector */
    status = VNA_STATUS_INIT_DSP;

    // Set BUSY animation to LED 3 to indicate that the FPGA is starting up
    set_led_state(2, LED_BUSY);

    // Initialize the FPGA
    init_FPGA_DSP();

#ifdef TEST_MODE
    // Wait a bit
    tx_thread_sleep(100);
#endif

    // Turn off the busy LED 3 to indicate that the ADC is ready
    set_led_state(2, LED_OFF);

    // Set status to idle
    status = VNA_STATUS_IDLE;

    vna_job current_job = VNA_JOB_NONE;

    // Busy wait for now
    while (true) {
        state = tx_queue_receive(&main_queue, &current_job, TX_WAIT_FOREVER);
        if (state != TX_SUCCESS) {
            // Handle error
            status = VNA_STATUS_ERROR;
        }
        switch (current_job) {
        case VNA_JOB_CALIBRATE:
            status = VNA_STATUS_CALIBRATING;
            vna_calibrate();
            break;
        case VNA_JOB_MEASURE:
            status = VNA_STATUS_MEASURING;
            vna_measure();
            break;
        case VNA_JOB_APPLY_CORRECTION:
            status = VNA_STATUS_APPLYING_CORRECTION;
            vna_refresh_meas_corrected();
            break;
        default:
            status = VNA_STATUS_ERROR;
        }

        tx_thread_sleep(10);
        // After finishing the current job, go back to idle
        status = VNA_STATUS_IDLE;
    }
}

/**
 * Get the current status of the VNA
 *
 * @return The current status of the VNA
 */
vna_status vna_get_status() {
    return status;
}

/**
 * Request a job to be executed by the VNA
 *
 * @param job The job to be executed
 * @return 0 on success, -1 on error
 */
int32_t vna_request_job(vna_job job) {
    // Send the job to the main thread
    return tx_queue_send(&main_queue, &job, TX_WAIT_FOREVER);
}

/**
 * Perform a calibration of the active measurement data set
 *
 * @return 0 on success, -1 if no active measurement data set is set, -10 or less if error applying calibration
 */
int32_t vna_measure_point(const uint64_t freq, meas_point_t* point) {
    // Start SPI measurement
    dsp_start_measurement(freq);

    // Wait for the CONVERSION_DONE interrupt to signal the event flag
    ULONG actual_flags;
    tx_event_flags_get(&measurement_event_flags, MEASUREMENT_EVENT_FLAG, TX_AND_CLEAR, &actual_flags, TX_WAIT_FOREVER);

    /* Read the measurement */
    dsp_read_point(point);
    return 0;
}

// External interrupt handler
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
    // Signal the event flag
    if (GPIO_Pin == CONVERSION_DONE_Pin)
        tx_event_flags_set(&measurement_event_flags, MEASUREMENT_EVENT_FLAG, TX_OR);
}
