//
// Created by matej on 02/09/2024.
//

#ifndef VNA_CORE_H
#define VNA_CORE_H

#include "main.h"

#include "vna_signaling.h"
#include "vna_calibration.h"
#include "vna_measurement.h"

#include "vna_stuw81300.h"
#include "vna_ads4222.h"

//#define TEST_MODE

#define MAIN_THREAD_STACK_SIZE 1024

// This needs to be visible from C code
#ifdef __cplusplus
extern "C" {
#endif

void vna_init();
[[noreturn]] VOID vna_thread_entry(ULONG initial_input);

#ifdef __cplusplus
}
#endif

#endif //VNA_CORE_H
