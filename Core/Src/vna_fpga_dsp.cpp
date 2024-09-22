//
// Created by matej on 18/09/2024.
//

#include "vna_fpga_dsp.h"

// TODO define the actual coefficients and their number
constexpr uint32_t DSP_DEFAULT_COEFF_NUM = 12;
int32_t DSP_DEFAULT_COEFFS[DSP_DEFAULT_COEFF_NUM] = {12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1};

void init_FPGA_DSP(){
    // Set coefficients to default values
    dsp_set_rbw_filter_coefficients(DSP_DEFAULT_COEFFS, DSP_DEFAULT_COEFF_NUM);
}

void send_DSP(uint32_t addr, uint32_t data) {
 	// TODO fix data width and address width to match the actual DSP
    // Ensure addr is within 4 bits and data is within 27 bits
    addr &= DSP_ADDR_MASK; // Mask to 4 bits
    data &= DSP_DATA_MASK; // Mask to 27 bits

    // Combine R/W (0 for write) + ADDR + DATA into a 32-bit variable
    uint32_t spi_data = (addr << DSP_ADDR_SHIFT) | data; // R/W is 0, so we skip adding it

    // Set LE pin low
    HAL_GPIO_WritePin(DSP_CS_FPGA_PORT, DSP_CS_FPGA_PIN, GPIO_PIN_RESET);

    // Tlc delay
    delay_us(DSP_SPI_DELAY);

    // Send data via SPI
    int stuw_spi_state = HAL_SPI_Transmit(&DSP_SPI, reinterpret_cast<uint8_t*>(&spi_data), DSP_SPI_DATA_SIZE, DSP_SPI_DELAY);

    if (stuw_spi_state != HAL_OK) {
        // STM32 Breakpoint
        delay_us(DSP_SPI_DELAY);
    }

    // Tcl delay
    delay_us(DSP_SPI_DELAY);

    // Set LE pin high briefly
    HAL_GPIO_WritePin(DSP_CS_FPGA_PORT, DSP_CS_FPGA_PIN, GPIO_PIN_SET);

    // Tdi delay
    delay_us(DSP_SPI_DELAY);
}

uint32_t read_DSP(uint32_t addr) {
    // Ensure addr is within 4 bits
    addr &= DSP_ADDR_MASK; // Mask to 4 bits

    // Construct the 32-bit read command: R/W bit (1 for read) + ADDR
    uint32_t spi_command = DSP_RW_MASK | (addr << DSP_ADDR_SHIFT);

    uint32_t spi_received_data = 0; // Variable to store received data

    // Set LE pin low
    HAL_GPIO_WritePin(DSP_CS_FPGA_PORT, DSP_CS_FPGA_PIN, GPIO_PIN_RESET);

    // Tlc delay
    delay_us(DSP_SPI_DELAY);

    // Send the read command via SPI and receive the data
    HAL_SPI_TransmitReceive(&DSP_SPI, reinterpret_cast<uint8_t*>(&spi_command),
                            reinterpret_cast<uint8_t*>(&spi_received_data), DSP_SPI_DATA_SIZE, DSP_SPI_TIMEOUT);

    // Tcl delay
    delay_us(DSP_SPI_DELAY);

    // Set LE pin high briefly
    HAL_GPIO_WritePin(DSP_CS_FPGA_PORT, DSP_CS_FPGA_PIN, GPIO_PIN_SET);

    // Tdi delay
    delay_us(DSP_SPI_DELAY);

    // Return the received 27-bit data, masking out the upper 5 bits (R/W and ADDR bits)
    return (spi_received_data & DSP_DATA_MASK);
}

int32_t dsp_set_rbw_filter_coefficient(const int32_t coefficient, const uint32_t coef_index) {
    // Check if the index is within bounds
    if (coef_index >= DSP_MAX_COEFFICIENTS)
        return -1;

    // TODO implement the actual coefficient setting

    return 0;
}

/**
 * Set multiple coefficients at once
 *
 * @param coefficients Array of coefficients
 * @param num_coefficients Number of coefficients
 * @return 0 on success, -1 if the number of coefficients is out of bounds
 */
int32_t dsp_set_rbw_filter_coefficients(const int32_t *coefficients, const uint32_t num_coefficients) {
    // Check if the number of coefficients is within bounds
    if (num_coefficients > DSP_MAX_COEFFICIENTS)
        return -1;

    // Set the coefficients one by one
    for (uint32_t i = 0; i < num_coefficients; i++)
        dsp_set_rbw_filter_coefficient(coefficients[i], i);

    return 0;
}

int32_t dsp_get_rbw_filter_coefficient(int32_t *coefficient, const uint32_t coef_index) {
    // Check if the index is within bounds
    if (coef_index >= DSP_MAX_COEFFICIENTS)
        return -1;

    int32_t read_coefficient = 0;

    // TODO implement the actual coefficient getting

    *coefficient = read_coefficient;

    return 0;
}

int32_t dsp_start_measurement(const uint64_t freq) {
    // TODO implement the actual DSP measurement start

    return 0;
}

int32_t dsp_read_point(meas_point_t *point) {
    // TODO implement the actual DSP point reading

    return 0;
}