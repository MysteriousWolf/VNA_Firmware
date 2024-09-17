//
// Created by matej on 04/09/2024.
//

#include "vna_ads4222.h"

/**
 * @brief Send a 16-bit packet (8-bit address + 8-bit data) to the ADS4222
 * @param addr: 8-bit address of the register to write to
 * @param data: 8-bit data to write to the register
 */
void send_ADS4222(uint16_t addr, uint16_t data) {
    /* Build the address and data */

    // Ensure addr and data are both within 8 bits
    addr &= ADS4222_ADDR_MASK;
    data &= ADS4222_DATA_MASK;

    // Combine ADDR (8 bit) + DATA (8 bit) into a 16-bit variable
    uint16_t spi_data = (addr << ADS4222_ADDR_SHIFT) | data;

    // Tie SEN low to start the communication
    HAL_GPIO_WritePin(ADC_SEN_PORT, ADC_SEN_PIN, GPIO_PIN_RESET);

    // Tsloads delay
    delay_us(ADC_SPI_DELAY);

    // Send data via SPI
    HAL_SPI_Transmit(&ADC_SPI, reinterpret_cast<uint8_t*>(&spi_data), ADC_SPI_DATA_SIZE, ADC_SPI_TIMEOUT);

    // Tsloadh delay
    delay_us(ADC_SPI_DELAY);

    // Tie SEN high to end the communication
    HAL_GPIO_WritePin(ADC_SEN_PORT, ADC_SEN_PIN, GPIO_PIN_SET);
    // Tie SCK high to make sure it isn't left floating in an unknown state
    //HAL_GPIO_WritePin(ADC_SCK_PORT, ADC_SCK_PIN, GPIO_PIN_SET);

    // Short additional delay to make consecutive writes more readable
    delay_us(ADC_SPI_DELAY*2);
}

/**
 * @brief Read a 16-bit packet (8-bit address + 8-bit data) from the ADS4222
 * @param addr: 8-bit address of the register to read from
 * @return 16-bit packet containing the padding and 8-bit data
 */
uint16_t read_ADS4222(uint16_t addr) {
    // do not allow reading from the 00 register (it messes up the readout register)
    if (addr == ADS4222_REG00) {
        return 0;
    }

    // readout register to 1
    send_ADS4222(ADS4222_REG00, REG00_READOUT_MASK);

    // write address to the register and read the second part of the data
    // Ensure addr is within 4 bits
    addr &= ADS4222_ADDR_MASK; // Mask to 4 bits

    // Construct the 16-bit read command (address + empty data)
    uint16_t spi_command = addr << ADS4222_ADDR_SHIFT;

    uint16_t spi_received_data = 0; // Variable to store received data

    // Tie SEN low to start the communication
    HAL_GPIO_WritePin(ADC_SEN_PORT, ADC_SEN_PIN, GPIO_PIN_RESET);

    // Tsloads delay
    delay_us(ADC_SPI_DELAY);

    // Send the read command via SPI and receive the data
    HAL_SPI_TransmitReceive(&ADC_SPI, reinterpret_cast<uint8_t*>(&spi_command), reinterpret_cast<uint8_t*>(&spi_received_data), ADC_SPI_DATA_SIZE, ADC_SPI_TIMEOUT);

    // Tsloadh delay
    delay_us(ADC_SPI_DELAY);

    // Tie SEN high to end the communication
    HAL_GPIO_WritePin(ADC_SEN_PORT, ADC_SEN_PIN, GPIO_PIN_SET);
    // Tie SCK high to make sure it isn't left floating in an unknown state
    //HAL_GPIO_WritePin(ADC_SCK_PORT, ADC_SCK_PIN, GPIO_PIN_SET);

    // Short delay before the next operation
    delay_us(ADC_SPI_DELAY*2);

    // readout register to 0
    send_ADS4222(ADS4222_REG00, 0);

    // Return the received data
    return spi_received_data & ADS4222_DATA_MASK;
}

/**
 * @brief Initialize the ADS4222 ADC with default settings for the VNA
 */
void init_ADS4222() {
    // Tie SEN low to set the default output format to CMOS
    HAL_GPIO_WritePin(ADC_SEN_PORT, ADC_SEN_PIN, GPIO_PIN_RESET);

    // Send a short reset pulse to the ADS4222 (this can be skipped if we use software reset)
    HAL_GPIO_WritePin(ADC_RST_PORT, ADC_RST_PIN, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(ADC_RST_PORT, ADC_RST_PIN, GPIO_PIN_RESET);

    // Keep RESET low and tie SEN high to enable serial communication
    HAL_GPIO_WritePin(ADC_SEN_PORT, ADC_SEN_PIN, GPIO_PIN_SET);

    /* Initialize registers */
    // Reset all registers
    send_ADS4222(ADS4222_REG00, REG00_RESET_MASK);
    // Short delay after reset
    delay_us(10);

    // High performance mode (recommended by the datasheet)
    send_ADS4222(ADS4222_REG03, REG03_HIGH_PERF_MODE_ON_MASK);
    // Output format to twos complement
    send_ADS4222(ADS4222_REG29, REG29_DATA_FORMAT_TWOS_COMPL_MASK);
    // Likely no need for offset correction for this application (this makes sure it's disabled)
    send_ADS4222(ADS4222_REG3D, 0);
    // CMOS output mode
    send_ADS4222(ADS4222_REG41, REG41_LVDS_CMOS_CMOS_MASK | REG41_CMOS_CLKOUT_STRENGTH_MAX_MASK);

    // Short delay after initialization to allow the ADC to stabilize
    delay_us(10);
}
