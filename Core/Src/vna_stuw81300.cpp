#include "vna_stuw81300.h"

// SCPI frequency units with their multipliers
scpi_choice_def_t scpi_choice_frequency_units[] = {
    {"Hz", 1},
    {"kHz", 1000},
    {"MHz", 1000000},
    {"GHz", 1000000000},
    SCPI_CHOICE_LIST_END
};

/** @brief Send data to the STUW81300 PLL
 *  @param addr The address to write to
 *  @param data The data to write to the address
 */
void send_STUW81300(uint32_t addr, uint32_t data) {
    // TODO Add chip select pin if needed
    // Ensure addr is within 4 bits and data is within 27 bits
    addr &= STUW81300_ADDR_MASK; // Mask to 4 bits
    data &= STUW81300_DATA_MASK; // Mask to 27 bits

    // Combine R/W (0 for write) + ADDR + DATA into a 32-bit variable
    uint32_t spi_data = (addr << STUW81300_ADDR_SHIFT) | data; // R/W is 0, so we skip adding it

    // Set LE pin low
    HAL_GPIO_WritePin(PLL_LE_PORT, PLL_LE_PIN, GPIO_PIN_RESET);

    // Tlc delay
    delay_us(PLL_SPI_DELAY);

    // Send data via SPI
    int stuw_spi_state = HAL_SPI_Transmit(&PLL_SPI, reinterpret_cast<uint8_t*>(&spi_data), PLL_SPI_DATA_SIZE, PLL_SPI_TIMEOUT);

    if (stuw_spi_state != HAL_OK) {
        // STM32 Breakpoint
        delay_us(PLL_SPI_DELAY);
    }

    // Pull CLK pin high to speed up the pullup process
    HAL_GPIO_WritePin(PLL_CLK_PORT, PLL_CLK_PIN, GPIO_PIN_SET);

    // Tcl delay
    delay_us(PLL_SPI_DELAY);

    // Set LE pin high briefly
    HAL_GPIO_WritePin(PLL_LE_PORT, PLL_LE_PIN, GPIO_PIN_SET);

    // Tdi delay
    delay_us(PLL_SPI_DELAY);

    // Set LE pin low again (maybe not needed?)
    //HAL_GPIO_WritePin(PLL_LE_PORT, PLL_LE_PIN, GPIO_PIN_RESET);
}

/** @brief Read data from the STUW81300 PLL
 *  @param addr The address to read from
 *  @return The data read from the address
 */
uint32_t read_STUW81300(uint32_t addr) {
    // Ensure addr is within 4 bits
    addr &= STUW81300_ADDR_MASK; // Mask to 4 bits

    // Construct the 32-bit read command: R/W bit (1 for read) + ADDR
    uint32_t spi_command = STUW81300_RW_MASK | (addr << STUW81300_ADDR_SHIFT);

    uint32_t spi_received_data = 0; // Variable to store received data

    // Set LE pin low
    HAL_GPIO_WritePin(PLL_LE_PORT, PLL_LE_PIN, GPIO_PIN_RESET);

    // Tlc delay
    delay_us(PLL_SPI_DELAY);

    // Send the read command via SPI and receive the data
    HAL_SPI_TransmitReceive(&PLL_SPI, reinterpret_cast<uint8_t*>(&spi_command),
                            reinterpret_cast<uint8_t*>(&spi_received_data), PLL_SPI_DATA_SIZE, PLL_SPI_TIMEOUT);

    // Pull CLK pin high to speed up the pullup process
    HAL_GPIO_WritePin(PLL_CLK_PORT, PLL_CLK_PIN, GPIO_PIN_SET);

    // Tcl delay
    delay_us(PLL_SPI_DELAY);

    // Set LE pin high briefly
    HAL_GPIO_WritePin(PLL_LE_PORT, PLL_LE_PIN, GPIO_PIN_SET);

    // Tdi delay
    delay_us(PLL_SPI_DELAY);

    // Set LE pin low again (maybe not needed?)
    //HAL_GPIO_WritePin(PLL_LE_PORT, PLL_LE_PIN, GPIO_PIN_RESET);

    // Return the received 27-bit data, masking out the upper 5 bits (R/W and ADDR bits)
    return (spi_received_data & STUW81300_DATA_MASK);
}

/* Default settings for the STUW81300 PLL */
// Set the ST9 register to 0
constexpr uint32_t ST9_INIT = 0;
// Set the REG_4V5 to 4.5V/3.3V depending on global settings
constexpr uint32_t ST8_INIT =
    (CORE_VCC_LOW_MASK & REG8_REG_VCO_4V5_VOUT_3_3V_MASK) |
    (CORE_VCC_HIGH_MASK & REG8_REG_VCO_4V5_VOUT_4_5V_MASK);
// Set the LD_SDO bit to 0 for CMOS output mode (may need REG7_LD_SDO_MODE_MASK if we decide to actually use the pin)
// Cycle slip compensation may be needed for faster frequency switching
constexpr uint32_t ST7_INIT = 0 | REG7_LD_SDO_tristate_MASK /*| REG7_LD_SDO_MODE_MASK*/;
// Enable temperature compensation (this one may need to be off for faster frequency switching - TBD)
// EN_AUTOCAL may be needed for better frequency accuracy
constexpr uint32_t ST6_INIT = REG6_CAL_TEMP_COMP_MASK;
// Set the RF2 bit to low power (for now, may need to be full power if needed for the SHF mixer)
constexpr uint32_t ST5_INIT = REG5_RF2_OUTBUF_LP_MASK;
// maybe enable mute on unlock?
// PFD settings are set as in the example
constexpr uint32_t ST4_INIT =
    (CORE_VCC_LOW_MASK & REG4_CALB_3V3_MODE1_MASK) |
    (CORE_VCC_LOW_MASK & REG4_RF_OUT_3V3_MASK) |
    (CORE_VCC_HIGH_MASK & (0b111 << REG4_VCO_AMP_OFFSET) & REG4_VCO_AMP_MASK) |
    (CORE_VCC_LOW_MASK & (0b010 << REG4_VCO_AMP_OFFSET) & REG4_VCO_AMP_MASK) |
    (REG4_CALB_3V3_MODE0_MASK & CORE_VCC_LOW_MASK) |
    REG4_REF_BUFF_MODE_SINGLE_MASK |
    // 10 ns lock detect precision (default is 2 ns 000)
    ((0b100 << REG4_LD_PREC_OFFSET) & REG4_LD_PREC_MASK) |
    // 1024 cycles lock detect count (for FPFD 50 MHz which is ~ 60MHz that we have);
    ((0b101 << REG4_LD_COUNT_OFFSET) & REG4_LD_COUNT_MASK);
constexpr uint32_t ST3_INIT = (DBR_MASK & REG3_DBR_MASK);
constexpr uint32_t ST2_INIT = (DBR_MASK & REG2_DBR_MASK);
constexpr uint32_t ST1_INIT = (DBR_MASK & REG1_DBR_MASK) | REG1_RF1_OUT_PD_MASK; // Power down RF1
constexpr uint32_t ST0_INIT = ((0b10011 << REG0_CP_SEL_OFFSET) & REG0_CP_SEL_MASK);
/* End of default settings */

/** @brief Initialize the STUW81300 PLL with the default settings
 *  @param busy_wait True to wait for the PLL to lock before returning
 */
void init_STUW81300(const bool busy_wait) {
    // Initialize the STUW81300 PLL
    send_STUW81300(STUW81300_REG9, ST9_INIT);
    send_STUW81300(STUW81300_REG8, ST8_INIT);
    send_STUW81300(STUW81300_REG7, ST7_INIT);
    send_STUW81300(STUW81300_REG6, ST6_INIT);
    send_STUW81300(STUW81300_REG5, ST5_INIT);

    // Set the top 5 registers accordingly - this also finishes the initialization
    set_frequency_STUW81300(8000000000); // Set the frequency to 8 GHz and mute (min needed frequency)

    // Wait for the frequency to stabilize
    while (busy_wait && !check_lock_STUW81300()) {
        // Wait for the PLL to lock
        delay_us(10); // Small delay to prevent busy waiting
    }
}

/** @brief Calculate the VCO frequency based on the PLL parameters
 *  @param f_ref The reference frequency in Hz
 *  @param r_div The R divider of the refference frequency
 *  @param mod The MOD value in the fractional-N divider
 *  @param n_frac The N_FRAC value of the fractional-N divider
 *  @param n_int The N_INT value of the integer-N divider
 *  @param vco_double_frequency True if the VCO frequency is doubled by the divider in the feedback loop
 *  @return The calculated VCO frequency in Hz
 */
uint64_t calculate_fvco_STUW81300(const uint64_t f_ref, const uint32_t r_div, const uint32_t mod, const uint32_t n_frac,
                                  const uint32_t n_int, const bool vco_double_frequency) {
    // If values are out of range, fail the operation
    if (!((n_frac < mod) && in_range(r_div, REG3_R_DIV_RANGE) && in_range(mod, REG2_MOD_RANGE) &&
        in_range(n_frac, REG1_N_FRAC_RANGE) && in_range(n_int, REG0_N_INT_RANGE)))
        return 0;

    // Calculate the VCO frequency based on the output frequency
    return f_ref * (vco_double_frequency ? 2 : 1) * (n_int * mod + n_frac) / r_div / mod;
}

/** @brief Set the frequency of the PLL in Hz for RF2 (our selected output because of the frequency doubler)
 *  @param frequency The desired frequency in Hz (if 0, keeps the previous frequency setting)
 *  @param mute Mute the PLL while setting the frequency
 *  @return The actual frequency set in Hz
 */
uint64_t update_STUW81300(const uint64_t frequency, const bool mute) {
    // Check if the frequency is 0 to keep the current frequency
    const bool keep_frequency = frequency == 0;

    // Check if the frequency is within the allowed range
    if (!(keep_frequency | in_range64(frequency, FREQ_RANGE)))
        return 0;

    // The VCO frequency is half the output frequency (RF2 has a frequency doubler)
    const uint64_t vco_freq = frequency / 2;

    /* Internal frequency checks */
    // Check if the VCO frequency is over 6 GHz (if it is, VCO frequency is doubled by the divider in the feedback loop)
    const bool vco_double_frequency = vco_freq >= REG1_N_FRAC_FREQ_EDGE;
    // Check if the VCO frequency is over 4.5 GHz (used to determine VCALB_MODE)
    const bool vcalb_mode_cond = (vco_freq > REG4_VCALB_MODE_EDGE) || FLAG_3V3;

    /* Calculate the PLL parameters */
    // Set the correct VCALB_MODE bit based on the VCO frequency
    const uint32_t REG4_VCALB_MODE = vcalb_mode_cond ? REG4_VCALB_MODE_MASK : 0;
    // Power down RF2 output if mute is requested
    const uint32_t REG2_RF2_OUT_PD = mute ? REG2_RF2_OUT_PD_MASK : 0;
    // Set the PLL_SEL bit if the VCO frequency is over 6 GHz
    const uint32_t REG1_PLL_SEL = vco_double_frequency ? REG1_PLL_SEL_MASK : 0;

    // Initialize PLL parameters (minimum values)

    // Set the R_DIV value to 1 (fixed value) - gives internal 60 MHz reference (close to 50 MHz in the datasheet)
    constexpr uint32_t R_DIV = 1;
    // Fixed at max for max frequency precision
    //uint32_t MOD = REG2_MOD_MAX; //TODO REMOVE
    constexpr uint32_t MOD = REG2_MOD_MAX;
    // "zero"-init Nint and Nfrac (this gives the minimum VCO frequency at the beginning)
    static uint32_t N_INT = REG0_N_INT_MIN;
    static uint32_t N_FRAC = REG1_N_FRAC_MIN;

    // Calculate actual Nint and Nfrac values
    if (!keep_frequency) {
        const uint64_t k_vco = PLL_REF_FREQ / R_DIV * (vco_double_frequency ? 2 : 1);
        N_INT = vco_freq / k_vco;
        N_FRAC = (vco_freq - N_INT * k_vco) * MOD / k_vco;
    }

    // check the actual frequency (x2 because of the frequency doubler on RF2)
    const uint64_t actual_frequency = 2 * calculate_fvco_STUW81300(
        PLL_REF_FREQ, R_DIV, MOD, N_FRAC, N_INT, vco_double_frequency);

    // If values are out of range (frequency is 0), fail the operation
    if (actual_frequency == 0)
        return 0;

    //TODO remove
    //MOD = REG2_MOD_MIN;
    //N_INT = REG0_N_INT_MIN;
    //N_FRAC = REG1_N_FRAC_MIN;

    // Actually set the registers
    send_STUW81300(STUW81300_REG4, ST4_INIT | REG4_VCALB_MODE);
    send_STUW81300(STUW81300_REG3, ST3_INIT | ((R_DIV << REG3_R_DIV_OFFSET) & REG3_R_DIV_MASK));
    send_STUW81300(STUW81300_REG2, ST2_INIT | ((MOD << REG2_MOD_OFFSET) & REG2_MOD_MASK) | REG2_RF2_OUT_PD);
    send_STUW81300(STUW81300_REG1, ST1_INIT | ((N_FRAC << REG1_N_FRAC_OFFSET) & REG1_N_FRAC_MASK) | REG1_PLL_SEL);
    send_STUW81300(STUW81300_REG0, ST0_INIT | ((N_INT << REG0_N_INT_OFFSET) & REG0_N_INT_MASK));

    // Return the actual frequency set
    return actual_frequency;
}

/**
 * @brief Set the frequency of the STUW81300 PLL. If the frequency is 0, the current frequency is kept.
 *
 * @param frequency The desired frequency in Hz
 * @return The actual frequency set on the PLL
 */
uint64_t set_frequency_STUW81300(const uint64_t frequency) {
    // Set the frequency without muting the RF2 output
    return update_STUW81300(frequency, false);
}

/**
 * @brief Mute or unmute the RF2 output. The frequency remains the same.
 *
 * @param mute true to mute the RF2 output, false to unmute. This does not change the frequency.
 */
void mute_STUW81300(const bool mute) {
    // Keep the frequency the same, but mute/unmute the RF2 output
    update_STUW81300(STUW81300_REG0, mute);
}

/**
 * @brief Check if the RF2 output is muted
 *
 * @return true if the RF2 output is muted, false otherwise. Also returns false before the initialization of the PLL.
 */
bool is_muted_STUW81300() {
    // check if the RF2 output is muted in the register
    return read_STUW81300(STUW81300_REG2) & REG2_RF2_OUT_PD_MASK;
}

/**
 * @brief Check if the PLL is locked
 *
 * @return true if the PLL is locked, false otherwise
 */
bool check_lock_STUW81300() {
    // Check if the PLL lock register is set
    return read_STUW81300(STUW81300_REG10) & REG10_LOCK_DET_MASK;
}
