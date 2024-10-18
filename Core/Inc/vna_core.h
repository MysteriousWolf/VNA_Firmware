//
// Created by matej on 02/09/2024.
//

#ifndef VNA_CORE_H
#define VNA_CORE_H

#include "main.h"

#include "vna_cmd_engine.h"

#include "vna_signaling.h"
#include "vna_calibration.h"
#include "vna_measurement.h"

#include "vna_stuw81300.h"
#include "vna_ads4222.h"
#include "vna_fpga_dsp.h"

//#define TEST_MODE

// Thread stack sizes
#define MAIN_THREAD_STACK_SIZE 1024
#define SIGNALING_THREAD_STACK_SIZE 1024

// Message queues
#define MAIN_QUEUE_STACK_SIZE  16

// Event flags
#define MEASUREMENT_EVENT_FLAG  (0b1 << 0)
#define READOUT_EVENT_FLAG      (0b1 << 1)
#define ADC_READOUT_EVENT_FLAG  (0b1 << 2)
#define DSP_DMA_SPI_DONE        (0b1 << 10)
#define ADC_DMA_SPI_DONE        (0b1 << 11)
#define PLL_DMA_SPI_DONE        (0b1 << 12)

// This needs to be visible from C code
#ifdef __cplusplus
extern "C" {
#endif
// VNA core jobs
typedef enum vna_job {
    VNA_JOB_NONE,
    VNA_JOB_CALIBRATE,
    VNA_JOB_MEASURE,
    VNA_JOB_APPLY_CORRECTION,
    VNA_JOB_READ_ADC,
} vna_job;

// VNA core status
typedef enum vna_status {
    VNA_STATUS_ERROR,
    VNA_STATUS_UNINITIALIZED,
    VNA_STATUS_INIT,
    VNA_STATUS_INIT_SIG,
    VNA_STATUS_INIT_MEAS,
    VNA_STATUS_INIT_CALIB,
    VNA_STATUS_INIT_CMD,
    VNA_STATUS_INIT_VCO,
    VNA_STATUS_INIT_ADC,
    VNA_STATUS_INIT_DSP,
    VNA_STATUS_IDLE,
    VNA_STATUS_CALIBRATING,
    VNA_STATUS_MEASURING,
    VNA_STATUS_APPLYING_CORRECTION,
    VNA_STATUS_READING_ADC,
} vna_status;

extern TX_EVENT_FLAGS_GROUP measurement_event_flags;

void vna_init();
[[noreturn]] VOID vna_thread_entry(ULONG initial_input);

vna_status vna_get_status();
int32_t vna_request_job(vna_job job);

#ifdef __cplusplus
}
#endif

#endif //VNA_CORE_H
