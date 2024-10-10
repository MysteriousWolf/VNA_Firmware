//
// Created by matej on 18/09/2024.
//

#include "vna_fpga_dsp.h"

// TODO define the actual coefficients and their number
constexpr uint32_t DSP_DEFAULT_COEFF_NUM = 12;
int32_t DSP_DEFAULT_COEFFS[DSP_DEFAULT_COEFF_NUM] = {12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1};

uint32_t dsp_status = 0;
uint32_t spi_state = 0;

void init_FPGA_DSP() {
    // Set all fpga pins as digital inputs to make them high impedance
    GPIO_InitTypeDef GPIO_InitStruct = {};

    // Configure CDONE as floating input
    GPIO_InitStruct.Pin = FPGA_CDONE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(FPGA_CDONE_GPIO_Port, &GPIO_InitStruct);

    // Configure CRESET as floating input
    GPIO_InitStruct.Pin = FPGA_CRESET_Pin;
    HAL_GPIO_Init(FPGA_CRESET_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = FPGA_CS_Pin;
    HAL_GPIO_Init(FPGA_CS_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = DSP_CS_FLASH_PIN;
    HAL_GPIO_Init(DSP_CS_FLASH_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = DSP_SCLK_PIN;
    HAL_GPIO_Init(DSP_SCLK_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = DSP_MISO_PIN;
    HAL_GPIO_Init(DSP_MISO_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = DSP_MOSI_PIN;
    HAL_GPIO_Init(DSP_MOSI_PORT, &GPIO_InitStruct);

    // Set FPGA reset pin as output
    GPIO_InitStruct.Pin = FPGA_CRESET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(FPGA_CRESET_GPIO_Port, &GPIO_InitStruct);

    // Reset FPGA configuration
    HAL_GPIO_WritePin(FPGA_CRESET_GPIO_Port, FPGA_CRESET_Pin, GPIO_PIN_RESET);

    // Wait for the FPGA to register the signal
    tx_thread_sleep(10);

    // Set FPGA config reset pin high to start configuration
    HAL_GPIO_WritePin(FPGA_CRESET_GPIO_Port, FPGA_CRESET_Pin, GPIO_PIN_SET);

    // Wait for the FPGA to be configured
    while (HAL_GPIO_ReadPin(FPGA_CDONE_GPIO_Port, FPGA_CDONE_Pin) == GPIO_PIN_RESET)
        tx_thread_sleep(10);

    // Reinit GPIO
    dsp_gpio_reinit();

    // Init SPI
    dsp_spi_init();

    // Set coefficients to default values
    // dsp_set_rbw_filter_coefficients(DSP_DEFAULT_COEFFS, DSP_DEFAULT_COEFF_NUM);

    while (true) {
        dsp_status = read_DSP(0b1111111);
        tx_thread_sleep(20);
        dsp_status = read_DSP(0b1100110);
        tx_thread_sleep(20);
        dsp_status = read_DSP(0b1010101);
        tx_thread_sleep(20);
        //send_DSP(0b1111111, 0);
        //tx_thread_sleep(20);
    }
}

void send_DSP(uint32_t addr, uint32_t data) {
    // TODO fix data width and address width to match the actual DSP
    // Ensure addr is within 4 bits and data is within 27 bits
    addr &= DSP_ADDR_MASK; // Mask to 4 bits
    data &= DSP_DATA_MASK; // Mask to 27 bits

    // Combine R/W (0 for write) + ADDR + DATA into a 32-bit variable
    const uint32_t spi_data = (addr << DSP_ADDR_SHIFT) | data; // R/W is 0, so we skip adding it

    uint8_t tx_data[4]; // 32 bits split into 4 bytes
    tx_data[0] = (spi_data >> 24) & 0xFF;
    tx_data[1] = (spi_data >> 16) & 0xFF;
    tx_data[2] = (spi_data >> 8) & 0xFF;
    tx_data[3] = spi_data & 0xFF;

    // Set NCS pin low
    HAL_GPIO_WritePin(DSP_CS_FPGA_PORT, DSP_CS_FPGA_PIN, GPIO_PIN_RESET);

    // Tlc delay
    delay_us(DSP_SPI_DELAY);

    // Send data via SPI
    spi_state = HAL_SPI_Transmit(&DSP_SPI, tx_data, DSP_SPI_DATA_SIZE, DSP_SPI_TIMEOUT);

    if (spi_state != HAL_OK) {
        // STM32 Breakpoint
        delay_us(DSP_SPI_DELAY);
    }

    // Tcl delay
    delay_us(DSP_SPI_DELAY);

    // Set NCS pin high
    HAL_GPIO_WritePin(DSP_CS_FPGA_PORT, DSP_CS_FPGA_PIN, GPIO_PIN_SET);

    // Tdi delay
    delay_us(DSP_SPI_DELAY);
}

//uint32_t primask_bit;

uint32_t read_DSP(uint32_t addr) {
    // Ensure addr is within 4 bits
    addr &= DSP_ADDR_MASK; // Mask to 4 bits

    // Construct the 32-bit read command: R/W bit (1 for read) + ADDR
    uint8_t tx_data[4] = {}; // 32 bits split into 4 bytes
    tx_data[0] = DSP_RW_MASK >> DSP_ADDR_SHIFT | addr;
    tx_data[1] = 0xFA;
    tx_data[2] = 0x9C;
    tx_data[3] = 0x34;

    uint32_t spi_received_data = 0; // Variable to store received data

    // Set NCS pin low
    HAL_GPIO_WritePin(DSP_CS_FPGA_PORT, DSP_CS_FPGA_PIN, GPIO_PIN_RESET);

    // Tlc delay
    delay_us(DSP_SPI_DELAY);

    uint8_t rx_data[4]; // Receiving 32 bits

    // Send the read command via SPI and receive the data
    spi_state = HAL_SPI_TransmitReceive(&DSP_SPI, tx_data, rx_data, DSP_SPI_DATA_SIZE, DSP_SPI_TIMEOUT);

    if (spi_state != HAL_OK) {
        // STM32 Breakpoint
        delay_us(DSP_SPI_DELAY);
    }

    spi_state = HAL_SPI_GetError(&DSP_SPI);
    if (spi_state != HAL_SPI_ERROR_NONE) {
        // Handle the specific error, log or break here for debugging
        delay_us(DSP_SPI_DELAY);
    }

    // Tcl delay
    delay_us(DSP_SPI_DELAY);

    // Enter threadx critical section


    // Set NCS pin high
    HAL_GPIO_WritePin(DSP_CS_FPGA_PORT, DSP_CS_FPGA_PIN, GPIO_PIN_SET);

    // Tdi delay
    delay_us(DSP_SPI_DELAY);

    // Convert back to uint32_t
    spi_received_data = (rx_data[0] << 24) | (rx_data[1] << 16) | (rx_data[2] << 8) | rx_data[3];

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
int32_t dsp_set_rbw_filter_coefficients(const int32_t* coefficients, const uint32_t num_coefficients) {
    // Check if the number of coefficients is within bounds
    if (num_coefficients > DSP_MAX_COEFFICIENTS)
        return -1;

    // Set the coefficients one by one
    for (uint32_t i = 0; i < num_coefficients; i++)
        dsp_set_rbw_filter_coefficient(coefficients[i], i);

    return 0;
}

int32_t dsp_get_rbw_filter_coefficient(int32_t* coefficient, const uint32_t coef_index) {
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

int32_t dsp_read_point(meas_point_t* point) {
    // TODO implement the actual DSP point reading

    return 0;
}
