//
// Created by matej on 06/09/2024.
//

#ifndef VNA_CALIBRATION_H
#define VNA_CALIBRATION_H

#include "main.h"

#include "vna_meas_data.h"
#include "vna_measurement.h"

// This needs to be visible from C code
#ifdef __cplusplus
extern "C" {
#endif

extern calib_data_t calib_data[CALIB_STORAGE_CNT];

void vna_calib_init();

// Setting setters
int32_t vna_set_active_calib(uint32_t calib_idx);
int32_t vna_set_calib_start_frequency(uint64_t freq);
int32_t vna_set_calib_stop_frequency(uint64_t freq);
int32_t vna_set_calib_frequency_range(uint64_t start_freq, uint64_t stop_freq);
int32_t vna_set_calib_point_count(uint32_t count);

int32_t vna_set_calib_to_match_meas(const meas_data_t *meas_data_set);

// Setting getters
uint64_t vna_get_calib_start_frequency();
uint64_t vna_get_calib_stop_frequency();
uint32_t vna_get_calib_point_count();
bool vna_is_calib_valid();
bool vna_is_calib_valid_for_meas(const meas_meta_t *meas_meta);

// Calibration data getters
int32_t vna_get_active_calib();
meas_meta_t *vna_get_active_calib_meta();

// Calibration enable
void vna_enable_calib(bool enable);
bool vna_is_calib_enabled();

// Calibration data application functions
int32_t vna_get_calib_point(uint64_t freq, meas_point_t *point);
int32_t vna_apply_calib_single(uint64_t freq, const meas_point_t *point, meas_point_t *point_corrected);
int32_t vna_apply_calib(const meas_data_t *meas_data_set, meas_data_t *meas_data_set_corrected);

int32_t vna_calibrate();

#ifdef __cplusplus
}
#endif

#endif //VNA_CALIBRATION_H
