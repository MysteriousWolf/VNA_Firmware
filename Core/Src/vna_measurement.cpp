//
// Created by matej on 06/09/2024.
//

#include "vna_measurement.h"

meas_data_t meas_data[MEAS_STORAGE_CNT] = {};
meas_data_t meas_data_corrected = {};

meas_data_t meas_data_outgoing = {};

int32_t active_meas = 0;

/**
 * Initialize the measurement data sets
 */
void vna_meas_init() {
    // Initialize calibration data sets for max frequency range with max steps
    for (auto& [meta, points] : meas_data) {
        meta.start_freq = MIN_FREQ;
        meta.stop_freq = MAX_FREQ;
        meta.num_points = MEAS_MAX_POINTS;
        meta.freq_step = (MAX_FREQ - MIN_FREQ) / (MEAS_MAX_POINTS - 1);
        meta.valid = false;
    }

    // Initialize the corrected data set
    meas_data_corrected.meta.start_freq = MIN_FREQ;
    meas_data_corrected.meta.stop_freq = MAX_FREQ;
    meas_data_corrected.meta.num_points = MEAS_MAX_POINTS;
    meas_data_corrected.meta.freq_step = (MAX_FREQ - MIN_FREQ) / (MEAS_MAX_POINTS - 1);
    meas_data_corrected.meta.valid = false;
}

/**
 * Set the active measurement data set
 *
 * @param meas_idx Index of the measurement data set
 * @return 0 on success, -1 if the index is out of bounds
 */
int32_t vna_set_active_meas(const uint32_t meas_idx) {
    if (meas_idx >= MEAS_STORAGE_CNT)
        return -1;

    active_meas = static_cast<int32_t>(meas_idx);

    return vna_refresh_meas_corrected();
}

/**
 * Set the start frequency of the active measurement data set
 *
 * @param freq Start frequency in Hz
 * @return 0 on success, -3 if no active measurement data set is set
 */
int32_t vna_set_meas_start_frequency(const uint64_t freq) {
    if (active_meas < 0)
        return -3;

    return vna_set_start_frequency(&meas_data[active_meas].meta, freq);
}

/**
 * Set the stop frequency of the active measurement data set
 *
 * @param freq Stop frequency in Hz
 * @return 0 on success, -3 if no active measurement data set is set
 */
int32_t vna_set_meas_stop_frequency(const uint64_t freq) {
    if (active_meas < 0)
        return -3;

    return vna_set_stop_frequency(&meas_data[active_meas].meta, freq);
}

/**
 * Set the frequency range of the active measurement data set
 *
 * @param start_freq Start frequency in Hz
 * @param stop_freq Stop frequency in Hz
 * @return 0 on success, -3 if no active measurement data set is set
 */
int32_t vna_set_meas_frequency_range(const uint64_t start_freq, const uint64_t stop_freq) {
    if (active_meas < 0)
        return -3;

    return vna_set_frequency_range(&meas_data[active_meas].meta, start_freq, stop_freq);
}

/**
 * Set the number of points of the active measurement data set
 *
 * @param count Number of points
 * @return 0 on success, -1 if no active measurement data set is set
 */
int32_t vna_set_meas_point_count(const uint32_t count) {
    // Check if there are any active calibration data sets
    if (active_meas < 0)
        return -3; // No active calibration data set

    return vna_set_point_count(&meas_data[active_meas].meta, MEAS_MAX_POINTS, count);
}

/**
 * Get the start frequency of the active measurement data set
 * @return Start frequency of the active measurement data set
 */
uint64_t vna_get_meas_start_frequency() {
    // Check if there are any active calibration data sets
    if (active_meas < 0)
        return 0; // No active calibration data set
    return meas_data[active_meas].meta.start_freq;
}

/**
 * Get the stop frequency of the active measurement data set
 * @return Stop frequency of the active measurement data set
 */
uint64_t vna_get_meas_stop_frequency() {
    // Check if there are any active calibration data sets
    if (active_meas < 0)
        return 0; // No active calibration data set
    return meas_data[active_meas].meta.stop_freq;
}

/**
 * Get the number of points of the active measurement data set
 * @return Number of points of the active measurement data set
 */
uint32_t vna_get_meas_point_count() {
    // Check if there are any active calibration data sets
    if (active_meas < 0)
        return 0; // No active calibration data set
    return meas_data[active_meas].meta.num_points;
}

/**
 * Check if the active measurement data set is valid
 * @return True if the active measurement data set is valid, false otherwise
 */
bool vna_is_meas_valid() {
    // Check if there are any active calibration data sets
    if (active_meas < 0)
        return false; // No active calibration data set
    return meas_data[active_meas].meta.valid;
}

/**
 * Get the active measurement data set
 *
 * @return Index of the active measurement data set
 */
int32_t vna_get_active_meas() {
    return active_meas;
}

/**
 * Get the active measurement metadata
 *
 * @return Pointer to the active measurement metadata, nullptr if no active measurement data set is set
 */
meas_meta_t* vna_get_active_meas_meta() {
    // Check if there are any active measurement data sets
    if (active_meas < 0)
        return nullptr; // No active measurement data set
    return &meas_data[active_meas].meta;
}

/**
 * Refresh the corrected measurement data set
 *
 * @return 0 on success, -1 if no active measurement data set is set, -10 or less if error applying calibration
 */
int32_t vna_refresh_meas_corrected() {
    // Attempt to apply correction to the currently active measurement set
    if (active_meas < 0)
        return -1; // No active measurement data set

    // Invalidate currently corrected data set
    meas_data_corrected.meta.valid = false;

    // Apply the calibration set
    const int32_t status = vna_apply_calib(&meas_data[active_meas], &meas_data_corrected);

    if (status < 0)
        return status - 10; // Error applying calibration

    return status; // Success
}

/**
 * Perform a measurement of the active measurement data set
 *
 * @return 0 on success, -1 if no active measurement data set is set, -2 if frequency range is out of bounds, -3 if start frequency is greater than stop frequency
 */
int32_t vna_measure() {
    // Check if there is an active calibration data set
    if (active_meas < 0)
        return -1; // No active calibration data set

    meas_data_t* to_measure = &meas_data[active_meas];

    // Check if the active calibration data set has a frequency range within limits of the system
    if (!in_range64(to_measure->meta.start_freq, FREQ_RANGE) ||
        !in_range64(to_measure->meta.stop_freq, FREQ_RANGE))
        return -2; // Frequency range out of bounds

    // Check if stop frequency is greater than start frequency
    if (to_measure->meta.start_freq > to_measure->meta.stop_freq)
        return -3; // Start frequency is greater than stop frequency

    int32_t status = 0;

    // Iterate over frequency steps and calibrate each point
    for (int i = 0; i < to_measure->meta.num_points; i++) {
        if (i == 0)
            // Calibrate the first point
            status = vna_measure_point(to_measure->meta.start_freq, &to_measure->points[i]);
        else if (i == to_measure->meta.num_points - 1)
            // Calibrate the last point
            status = vna_measure_point(to_measure->meta.stop_freq, &to_measure->points[i]);
        else
            // Calibrate the rest of the points
            status = vna_measure_point(to_measure->meta.start_freq + i * to_measure->meta.freq_step,
                                       &to_measure->points[i]);

        // Stop calibration if an error occurs
        if (status < 0)
            break;
    }

    if (status == 0)
        to_measure->meta.valid = true;

    return vna_refresh_meas_corrected();
}
