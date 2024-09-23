//
// Created by matej on 06/09/2024.
//

#include "vna_calibration.h"

calib_data_t calib_data[CALIB_STORAGE_CNT] = {};

int32_t active_calib = 0;

bool correction_enabled = true;

/**
 * Initialize calibration data sets
 */
void vna_calib_init() {
    // Initialize calibration data sets for max frequency range with max steps
    for (auto& [meta, points] : calib_data) {
        meta.start_freq = MIN_FREQ;
        meta.stop_freq = MAX_FREQ;
        meta.num_points = CALIB_MAX_POINTS;
        meta.freq_step = (MAX_FREQ - MIN_FREQ) / CALIB_MAX_POINTS;
        meta.valid = false;
    }
}

/**
 * Set the active calibration data set
 * @param calib_idx Index of the calibration data set
 * @return Index of the active calibration data set (-1 if invalid index, -10 or more if error)
 */
int32_t vna_set_active_calib(const uint32_t calib_idx) {
    if (calib_idx >= CALIB_STORAGE_CNT)
        return -1;

    active_calib = static_cast<int32_t>(calib_idx);

    // Attempt to apply calibration to the current measuremenet set
    if (const int32_t status = vna_refresh_meas_corrected(); status < 0)
        return status;

    return active_calib;
}

/**
 * Set the start frequency of the active calibration data set
 * @param freq Start frequency in Hz
 * @return 0 if successful, negative error code otherwise
 */
int32_t vna_set_calib_start_frequency(const uint64_t freq) {
    if (active_calib < 0)
        return -3; // No active calibration data set

    return vna_set_start_frequency(&calib_data[active_calib].meta, freq);
}

/**
 * Set the stop frequency of the active calibration data set
 * @param freq Stop frequency in Hz
 * @return 0 if successful, negative error code otherwise
 */
int32_t vna_set_calib_stop_frequency(const uint64_t freq) {
    if (active_calib < 0)
        return -3; // No active calibration data set

    return vna_set_stop_frequency(&calib_data[active_calib].meta, freq);
}

/**
 * Set the frequency range of the active calibration data set
 * @param start_freq Start frequency in Hz
 * @param stop_freq Stop frequency in Hz
 * @return 0 if successful, negative error code otherwise
 */
int32_t vna_set_calib_frequency_range(const uint64_t start_freq, const uint64_t stop_freq) {
    if (active_calib < 0)
        return -3; // No active calibration data set

    return vna_set_frequency_range(&calib_data[active_calib].meta, start_freq, stop_freq);
}

/**
 * Set the number of calibration points for the active calibration data set
 * @param count Number of calibration points
 * @return 0 if successful, negative error code otherwise
 */
int32_t vna_set_calib_point_count(const uint32_t count) {
    // Check if there are any active calibration data sets
    if (active_calib < 0)
        return -3; // No active calibration data set

    return vna_set_point_count(&calib_data[active_calib].meta, CALIB_MAX_POINTS, count);
}

/**
 * Set the active calibration data set to match the metadata of a measurement data set
 * @param meas_data_set Pointer to the measurement data set
 * @return 0 if successful, negative error code otherwise
 */
int32_t vna_set_calib_to_match_meas(const meas_data_t* meas_data_set) {
    // Check if there are any active calibration data sets
    if (active_calib < 0)
        return -3; // No active calibration data set

    calib_data_t* calib = &calib_data[active_calib];

    // Clone the metadata from the measurement data set
    calib->meta = meas_data_set->meta;

    // Make sure the number of points is within the calibration data set limits
    if (calib->meta.num_points > CALIB_MAX_POINTS)
        calib->meta.num_points = CALIB_MAX_POINTS;

    // Re-set point count using the metadata methods to ensure consistency and recalculate steps
    return vna_set_point_count(&calib->meta, CALIB_MAX_POINTS, calib->meta.num_points);
}

/**
 * Get the start frequency of the active calibration data set
 * @return Start frequency in Hz
 */
uint64_t vna_get_calib_start_frequency() {
    // Check if there are any active calibration data sets
    if (active_calib < 0)
        return 0; // No active calibration data set
    return calib_data[active_calib].meta.start_freq;
}

/**
 * Get the stop frequency of the active calibration data set
 * @return Stop frequency in Hz
 */
uint64_t vna_get_calib_stop_frequency() {
    // Check if there are any active calibration data sets
    if (active_calib < 0)
        return 0; // No active calibration data set
    return calib_data[active_calib].meta.stop_freq;
}

/**
 * Get the number of calibration points of the active calibration data set
 * @return Number of calibration points
 */
uint32_t vna_get_calib_point_count() {
    // Check if there are any active calibration data sets
    if (active_calib < 0)
        return 0; // No active calibration data set
    return calib_data[active_calib].meta.num_points;
}

/**
 * Check if the currently active calibration set is valid
 * @return True if the calibration data set is valid, false otherwise
 */
bool vna_is_calib_valid() {
    // Check if there are any active calibration data sets
    if (active_calib < 0)
        return false; // No active calibration data set
    return calib_data[active_calib].meta.valid;
}

/**
 * Check if the currently active calibration set is valid for a given measurement data set
 * @param meas_meta Pointer to the measurement metadata
 * @return True if the calibration data set is valid for the measurement data set, false otherwise
 */
bool vna_is_calib_valid_for_meas(const meas_meta_t *meas_meta) {
    // Check if there are any active calibration data sets
    if (active_calib < 0)
        return false; // No active calibration data set

    // Get the current calibration data set
    const calib_data_t* calib = &calib_data[active_calib];

    // Check if calibration data is valid
    if (!calib->meta.valid)
        return false;

    // Check if the measurement frequency range is within the calibration frequency range
    if (meas_meta->start_freq < calib->meta.start_freq || meas_meta->stop_freq > calib->meta.stop_freq)
        return false;

    // All checks passed, calibration data is valid for the measurement data set
    return true;
}

/**
 * Get the active calibration data set
 * @return Index of the active calibration data set
 */
int32_t vna_get_active_calib() {
    return active_calib;
}

/**
 * Get the metadata of the active calibration data set
 * @return Pointer to the metadata of the active calibration data set
 */
meas_meta_t *vna_get_active_calib_meta() {
    // Check if there are any active calibration data sets
    if (active_calib < 0)
        return nullptr; // No active calibration data set
    return &calib_data[active_calib].meta;
}

/**
 * Enable or disable calibration correction
 * @param enable True to enable, false to disable
 * @return 0 if successful, negative error code otherwise
 */
void vna_enable_calib(const bool enable) {
    correction_enabled = enable;
}

/**
 * Check if calibration correction is enabled
 * @return True if calibration correction is enabled, false otherwise
 */
bool vna_is_calib_enabled() {
    return correction_enabled;
}

/**
 * Get the calibration point for any given frequency within the currently active calibration data set
 * @param freq Frequency in Hz
 * @param point Pointer to the output meas_point_t structure
 * @return 0 if successful, negative error code otherwise
 */
int32_t vna_get_calib_point(const uint64_t freq, meas_point_t* point) {
    if (active_calib < 0)
        return -1; // No active calibration data set

    if (!calib_data[active_calib].meta.valid)
        return -2; // Invalid calibration data set

    const calib_data_t* calib = &calib_data[active_calib];

    if (freq < calib->meta.start_freq || freq > calib->meta.stop_freq)
        return -3; // Frequency out of bounds

    // Forward prediction: Estimate the index based on integer math
    const int estimated_idx = static_cast<int>((freq - calib->meta.start_freq) / calib->meta.freq_step);
    int low, high;

    // Check if the estimated index is exactly correct
    if (calib->meta.start_freq + estimated_idx * calib->meta.freq_step == freq) {
        point->phase = calib->points[estimated_idx].phase;
        point->amplitude = calib->points[estimated_idx].amplitude;
        return 0; // Indicate that the frequency was found exactly
    }

    // Check if the current index is higher or lower than the target frequency
    if (calib->meta.start_freq + estimated_idx * calib->meta.freq_step < freq) {
        // The estimated index is below the target frequency
        low = estimated_idx;
        high = estimated_idx + 1;
    }
    else {
        // The estimated index is above the target frequency
        low = estimated_idx - 1;
        high = estimated_idx;
    }

    // Should not happen due to earlier bounds check
    if (low < 0 || high >= calib->meta.num_points)
        return -4; // Estimation error - out of bounds

    // Calculate the corresponding frequency for the low index
    const uint64_t freq_low = calib->meta.start_freq + low * calib->meta.freq_step;

    // Linear interpolation using integer math
    const uint64_t delta_phase = calib->points[high].phase - calib->points[low].phase;
    const uint64_t delta_amplitude = calib->points[high].amplitude - calib->points[low].amplitude;

    // Avoid division by zero (should not happen in normal cases)
    if (low == high) {
        point->phase = calib->points[low].phase;
        point->amplitude = calib->points[low].amplitude;
        return 0; // Indicate that the frequency was found exactly - and somehow missed in the estimation
    }

    // Linear interpolation
    point->phase = calib->points[low].phase + (delta_phase * (freq - freq_low)) / calib->meta.freq_step;
    point->amplitude = calib->points[low].amplitude + (delta_amplitude * (freq - freq_low)) / calib->meta.freq_step;

    return 1; // Indicate that the frequency was interpolated
}

/**
 * Apply calibration to a single measurement point (inefficient)
 * @param freq Frequency in Hz
 * @param point Pointer to the input meas_point_t structure
 * @param point_corrected Pointer to the output meas_point_t structure
 * @return 0 if successful, negative error code otherwise
 */
int32_t vna_apply_calib_single(const uint64_t freq, const meas_point_t* point, meas_point_t* point_corrected) {
    meas_point_t calib_point;
    const int32_t ret = vna_get_calib_point(freq, &calib_point);

    if (ret < 0)
        return ret; // Error in getting the calibration point

    // Apply the calibration
    point_corrected->phase = point->phase - calib_point.phase;
    point_corrected->amplitude = point->amplitude - calib_point.amplitude;

    return 0;
}

/**
 * Apply calibration to a whole measurement data set (more efficient)
 * @param meas_data_set Pointer to the input meas_data_t structure
 * @param meas_data_set_corrected Pointer to the output meas_data_t structure
 * @return 0 if successful, negative error code otherwise
 */
int32_t vna_apply_calib(const meas_data_t* meas_data_set, meas_data_t* meas_data_set_corrected) {
    if (active_calib < 0)
        return -1; // No active calibration data set

    if (!calib_data[active_calib].meta.valid)
        return -2; // Invalid calibration data set

    const calib_data_t* calib = &calib_data[active_calib];

    // Start frequency out of calibration range
    if (meas_data_set->meta.start_freq < calib->meta.start_freq)
        return -3; // Start frequency lower than in calibration data set

    // Stop frequency out of calibration range
    if (meas_data_set->meta.stop_freq > calib->meta.stop_freq)
        return -4; // Stop frequency higher than in calibration data set

    // Transfer unchanged metadata (except for the valid flag)
    meas_meta_t new_meta = meas_data_set->meta;
    new_meta.valid = false;
    meas_data_set_corrected->meta = new_meta;

    // Apply calibration to each point without using the get_calib_point function to improve efficiency
    // We use the fact that the calibration data is equally spaced and strictly monotonic in frequency
    // to speed up the high/low frequency search and linear interpolation

    // We know the first index of the calib and meas sets match exactly
    int calib_idx = 0;
    uint64_t low_freq = calib->meta.start_freq;
    uint64_t high_freq = calib->meta.start_freq + calib->meta.freq_step;

    // Iterate over measurement points and apply calibration using integer linear interpolation
    for (int i = 0; i < meas_data_set->meta.num_points; i++) {
        // Calculate the corresponding frequency for the current measurement point
        const uint64_t meas_freq = meas_data_set->meta.start_freq + i * meas_data_set->meta.freq_step;

        // Move calibration index forward if needed (never backwards)
        while (high_freq < meas_freq && calib_idx < calib->meta.num_points - 1) {
            calib_idx++;
            low_freq = high_freq;
            high_freq += calib->meta.freq_step;
        }

        // If indexes match exactly, no interpolation is needed
        if (high_freq == meas_freq) {
            meas_data_set_corrected->points[i].phase = meas_data_set->points[i].phase - calib->points[calib_idx].phase;
            meas_data_set_corrected->points[i].amplitude = meas_data_set->points[i].amplitude - calib->points[calib_idx]
                .amplitude;
            continue;
        }

        // Linear interpolation using integer math
        // Calculate the proportion of the difference between the two calibration points
        const uint64_t freq_diff = meas_freq - low_freq;

        // Interpolating phase
        meas_data_set_corrected->points[i].phase = meas_data_set->points[i].phase -
        (calib->points[calib_idx - 1].phase + (freq_diff * (calib->points[calib_idx].phase - calib->points[calib_idx
            - 1].phase)) / calib->meta.freq_step);

        // Interpolating amplitude
        meas_data_set_corrected->points[i].amplitude = meas_data_set->points[i].amplitude -
        (calib->points[calib_idx - 1].amplitude + (freq_diff * (calib->points[calib_idx].amplitude - calib->points[
            calib_idx - 1].amplitude)) / calib->meta.freq_step);
    }

    // Everything went well, mark the corrected data set as valid
    meas_data_set_corrected->meta.valid = true;

    return 0;
}

// Calibrate the selected frequency range
int32_t vna_calibrate() {
    // Check if there is an active calibration data set
    if (active_calib < 0)
        return -1; // No active calibration data set

    calib_data_t* to_calibrate = &calib_data[active_calib];

    // Check if the active calibration data set has a frequency range within limits of the system
    if (!in_range64(to_calibrate->meta.start_freq, FREQ_RANGE) ||
        !in_range64(to_calibrate->meta.stop_freq, FREQ_RANGE))
        return -2; // Frequency range out of bounds

    // Check if stop frequency is greater than start frequency
    if (to_calibrate->meta.start_freq > to_calibrate->meta.stop_freq)
        return -3; // Start frequency is greater than stop frequency

    int32_t status = 0;

    // Iterate over frequency steps and calibrate each point
    for (int i = 0; i < to_calibrate->meta.num_points; i++) {
        if (i == 0)
            // Calibrate the first point
            status = vna_measure_point(to_calibrate->meta.start_freq, &to_calibrate->points[i]);
        else if (i == to_calibrate->meta.num_points - 1)
            // Calibrate the last point
            status = vna_measure_point(to_calibrate->meta.stop_freq, &to_calibrate->points[i]);
        else
            // Calibrate the intermediate points
            status = vna_measure_point(to_calibrate->meta.start_freq + i * to_calibrate->meta.freq_step,
                                         &to_calibrate->points[i]);

        // Stop calibration if an error occurs
        if (status < 0)
            break;
    }

    if (status == 0)
        to_calibrate->meta.valid = true;

    return status;
}
