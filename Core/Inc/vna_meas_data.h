//
// Created by matej on 08/09/2024.
//

#ifndef VNA_MEAS_DATA_H
#define VNA_MEAS_DATA_H

#include "main.h"
#include "vna_stuw81300.h"

// Global measurement data constraints
#define MEAS_TYPE               int16_t    // Type of measurement data (16 bits should be more than enough)

#define CALIB_STORAGE_CNT       5           // Number of calibration data sets
#define CALIB_MAX_POINTS        500         // Maximum number of calibration points

#define MEAS_STORAGE_CNT        2           // Number of measurement data sets
#define MEAS_MAX_POINTS         5000       // Maximum number of measurement points

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

// Calibration is done from the first to the last frequency (both included) and equally spaced in between
typedef struct {
    meas_meta_t meta;
    meas_point_t points[CALIB_MAX_POINTS];
} calib_data_t;

// Measurement is done from the first to the last frequency (both included) and equally spaced in between
typedef struct {
    meas_meta_t meta;
    meas_point_t points[MEAS_MAX_POINTS];
} meas_data_t;

int32_t vna_set_start_frequency(meas_meta_t *meta, uint64_t freq);
int32_t vna_set_stop_frequency(meas_meta_t *meta, uint64_t freq);
int32_t vna_set_frequency_range(meas_meta_t *meta, uint64_t start_freq, uint64_t stop_freq);

int32_t vna_set_point_count(meas_meta_t *meta, uint32_t max_points, uint32_t count);

// This needs to be defined by the user of this library (used by both calib and meas to acquire data points)
int32_t vna_measure_point(uint64_t freq, meas_point_t *point);

#ifdef __cplusplus
}
#endif
#endif //VNA_MEAS_DATA_H
