//
// Created by matej on 06/09/2024.
//

#ifndef VNA_CALIBRATION_H
#define VNA_CALIBRATION_H
#include "main.h"
#include "vna_measurement.h"

#define CALIB_STORAGE_CNT       3           // Number of calibration data sets
#define CALIB_MAX_POINTS        500         // Maximum number of calibration points

// This needs to be visible from C code
#ifdef __cplusplus
extern "C" {
#endif

// Calibration is done from the first to the last frequency (both included) and equally spaced in between
typedef struct {
    meas_meta_t meta;
    meas_point_t points[CALIB_MAX_POINTS];
} calib_data_t;

extern calib_data_t calib_data[CALIB_STORAGE_CNT];

void vna_calib_init();

int32_t vna_set_active_calib(uint32_t calib_idx);
int32_t vna_set_calib_start_frequency(uint64_t freq);
int32_t vna_set_calib_stop_frequency(uint64_t freq);
int32_t vna_set_calib_frequency_range(uint64_t start_freq, uint64_t stop_freq);
int32_t vna_set_calib_point_count(meas_meta_t *meta, uint32_t count);

int32_t vna_get_active_calib();

int32_t vna_get_calib_point(uint64_t freq, meas_point_t *point);

int32_t vna_apply_calib_single(uint64_t freq, const meas_point_t *point, meas_point_t *point_corrected);
int32_t vna_apply_calib(const meas_data_t *meas_data_set, meas_data_t *meas_data_set_corrected);

// This needs to be defined by the user of this library
int32_t vna_calibrate_point(uint64_t freq, meas_point_t *point);

int32_t vna_calibrate();

#ifdef __cplusplus
}
#endif

#endif //VNA_CALIBRATION_H
