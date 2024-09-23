//
// Created by matej on 08/09/2024.
//

#include "vna_meas_data.h"

/**
 * @brief Set the start frequency of the measurement metadata
 *
 * @param meta Pointer to the measurement metadata
 * @param freq Start frequency in Hz
 * @return int32_t 0 if successful, -1 if frequency is out of range, -2 if start frequency is more than stop frequency
 */
int32_t vna_set_start_frequency(meas_meta_t* meta, const uint64_t freq) {
    // Frequency is out of range
    if (!in_range64(freq, FREQ_RANGE))
        return -1;

    // Start frequency is greater than stop frequency
    if (meta->stop_freq < freq)
        return -2;

    meta->start_freq = freq;

    // Estimate frequency step using integer math
    meta->freq_step = (meta->stop_freq - meta->start_freq) / (meta->num_points - 1);

    // The calibration data is now invalid for the new frequency range
    meta->valid = false;

    return 0;
}

/**
 * @brief Set the stop frequency of the measurement metadata
 *
 * @param meta Pointer to the measurement metadata
 * @param freq Stop frequency in Hz
 * @return int32_t 0 if successful, -1 if frequency is out of range, -2 if stop frequency is less than start frequency
 */
int32_t vna_set_stop_frequency(meas_meta_t* meta, const uint64_t freq) {
    // Frequency is out of range
    if (!in_range64(freq, FREQ_RANGE))
        return -1;

    // Stop frequency is less than start frequency
    if (meta->start_freq > freq)
        return -2;

    meta->stop_freq = freq;

    // Estimate frequency step using integer math
    meta->freq_step = (meta->stop_freq - meta->start_freq) / (meta->num_points - 1);

    // The data is now invalid for the new frequency range
    meta->valid = false;

    return 0;
}

/**
 * @brief Set the frequency range of the measurement metadata
 *
 * @param meta Pointer to the measurement metadata
 * @param start_freq Start frequency in Hz
 * @param stop_freq Stop frequency in Hz
 * @return int32_t 0 if successful, -1 if frequency is out of range, -2 if start frequency is more than stop frequency
 */
int32_t vna_set_frequency_range(meas_meta_t *meta, const uint64_t start_freq, const uint64_t stop_freq) {
    // Frequency is out of range
    if (!in_range64(start_freq, FREQ_RANGE) || !in_range64(stop_freq, FREQ_RANGE))
        return -1;

    // Start frequency is greater than stop frequency
    if (start_freq > stop_freq)
        return -2;

    meta->start_freq = start_freq;
    meta->stop_freq = stop_freq;

    // Estimate frequency step using integer math
    meta->freq_step = (meta->stop_freq - meta->start_freq) / (meta->num_points - 1);

    // The data is now invalid for the new frequency range
    meta->valid = false;

    return 0;
}

/**
 * @brief Set the number of points in the measurement metadata
 *
 * @param meta Pointer to the measurement metadata
 * @param max_points Maximum number of points
 * @param count Number of points
 * @return int32_t 0 if successful, -1 if number of points is out of range, -2 if number of points is less than 2
 */
int32_t vna_set_point_count(meas_meta_t *meta, const uint32_t max_points, const uint32_t count) {
    // Number of points is out of range
    if (count > max_points)
        return -1;

    // Number of points is less than 2
    if (count < 2)
        return -2;

    // Set the number of points
    meta->num_points = count;

    // Estimate frequency step using integer math
    meta->freq_step = (meta->stop_freq - meta->start_freq) / (meta->num_points - 1);

    // The data is now invalid for the new frequency range
    meta->valid = false;

    // Return success
    return 0;
}
