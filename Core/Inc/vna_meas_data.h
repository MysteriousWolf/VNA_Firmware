//
// Created by matej on 08/09/2024.
//

#ifndef VNA_MEAS_DATA_H
#define VNA_MEAS_DATA_H
#include "main.h"
#include "vna_stuw81300.h"

#define MEAS_TYPE               int16_t    // Type of measurement data (16 bits should be more than enough)

// This needs to be visible from C code
#ifdef __cplusplus
extern "C" {
#endif

// Measurement point
typedef struct {
    MEAS_TYPE phase;
    MEAS_TYPE amplitude;
} meas_point_t;

// Metadata for measurement data
typedef struct {
    bool valid;
    uint64_t start_freq;
    uint64_t stop_freq;
    uint64_t freq_step; // Estimated frequency step
    uint16_t num_points;
} meas_meta_t;

int32_t vna_set_start_frequency(meas_meta_t *meta, uint64_t freq);
int32_t vna_set_stop_frequency(meas_meta_t *meta, uint64_t freq);
int32_t vna_set_frequency_range(meas_meta_t *meta, uint64_t start_freq, uint64_t stop_freq);

#ifdef __cplusplus
}
#endif
#endif //VNA_MEAS_DATA_H
