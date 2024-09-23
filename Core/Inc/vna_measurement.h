//
// Created by matej on 06/09/2024.
//

#ifndef VNA_MEASUREMENT_H
#define VNA_MEASUREMENT_H

#include "main.h"

#include "vna_meas_data.h"
#include "vna_calibration.h"

// This needs to be visible from C code
#ifdef __cplusplus
extern "C" {
#endif

extern meas_data_t meas_data[MEAS_STORAGE_CNT];
extern meas_data_t meas_data_corrected;

void vna_meas_init();

// Setting setters
int32_t vna_set_active_meas(uint32_t meas_idx);
int32_t vna_set_meas_start_frequency(uint64_t freq);
int32_t vna_set_meas_stop_frequency(uint64_t freq);
int32_t vna_set_meas_frequency_range(uint64_t start_freq, uint64_t stop_freq);
int32_t vna_set_meas_point_count(meas_meta_t *meta, uint32_t count);

// Setting getters
uint64_t vna_get_meas_start_frequency();
uint64_t vna_get_meas_stop_frequency();
uint32_t vna_get_meas_point_count();
bool vna_get_meas_valid();

// Measurement data getters
int32_t vna_get_active_meas();
meas_meta_t *vna_get_active_meas_meta();

int32_t vna_refresh_meas_corrected();

// This needs to be defined by the user of this library
int32_t vna_measure_point(uint64_t freq, meas_point_t *point);

int32_t vna_measure();

#ifdef __cplusplus
}
#endif
#endif //VNA_MEASUREMENT_H
