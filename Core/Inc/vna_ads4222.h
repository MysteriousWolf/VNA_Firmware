//
// Created by matej on 04/09/2024.
//

#ifndef VNA_ADS4222_H
#define VNA_ADS4222_H

#include "main.h"
#include "vna_tx_utils.h"

// This driver is for the Texas Instruments ADS4222 ADC (Dual-Channel, 12-Bit, 65-MSPS)
// It should also work with any ADC from the ADS42xx family according to the datasheet.
// Product page: https://www.ti.com/product/ADS4222
// Datasheet: https://www.ti.com/lit/ds/symlink/ads4222.pdf

// General properties
#define ADS4222_NAME    "ADS4222"  // Device name
#define ADS4222_FREQ    65000000    // ADC sample rate (65 MSPS)
#define ADS4222_BITS    12          // ADC resolution (12 bits)

// SPI settings (use SPI hspi3 on H5)
#define ADC_SPI         hspi3
#define ADC_SPI_INST    SPI3
#define ADC_SEN_PORT    SPI3_CS_GPIO_Port
#define ADC_SEN_PIN     SPI3_CS_Pin
#define ADC_RST_PORT    SPI3_RST_GPIO_Port
#define ADC_RST_PIN     SPI3_RST_Pin
#define ADC_SCK_PORT    SPI3_SCK_GPIO_Port
#define ADC_SCK_PIN     SPI3_SCK_Pin

#define ADC_SPI_DELAY       10  // in us, should be enough
#define ADC_SPI_DATA_SIZE   1   // 1x16-bit SPI data size
#define ADC_SPI_TIMEOUT     5   // in ms, should be enough

// Register component shifts and masks
#define ADS4222_ADDR_MASK         0xFF                                            // Address bits before shifting (bits 7 to 0)
#define ADS4222_ADDR_SHIFT        8                                               // Shift for address bits
#define ADS4222_ADDR_MASK_SH      (STUW81300_ADDR_MASK << STUW81300_ADDR_SHIFT)   // Address bits (bits 15 to 8)
#define ADS4222_DATA_MASK         0xFF                                            // Data bits (bits 7 to 0)

// Register addresses
#define ADS4222_REG00        0x00      // RESET and READOUT
#define ADS4222_REG01        0x01      // LVDS SWING
#define ADS4222_REG03        0x03      // HIGH PERF MODE
#define ADS4222_REG25        0x25      // CH A GAIN, CH A TEST PATTERNS
#define ADS4222_REG29        0x29      // DATA FORMAT
#define ADS4222_REG2B        0x2B      // CH B GAIN, CH B TEST PATTERNS
#define ADS4222_REG3D        0x3D      // ENABLE OFFSET CORR
#define ADS4222_REG3F        0x3F      // CUSTOM PATTERN D[13:8]
#define ADS4222_REG40        0x40      // CUSTOM PATTERN D[7:0]
#define ADS4222_REG41        0x41      // LVDS CMOS, CMOS CLKOUT STRENGTH, DIS OBUF
#define ADS4222_REG42        0x42      // CLKOUT FALL POSN, CLKOUT RISE POSN, EN DIGITAL
#define ADS4222_REG45        0x45      // STBY, LVDS CLKOUT STRENGTH, LVDS DATA STRENGTH, PDN GLOBAL
#define ADS4222_REG4A        0x4A      // HIGH FREQ MODE CH B
#define ADS4222_REG58        0x58      // HIGH FREQ MODE CH A
#define ADS4222_REGBF        0xBF      // CH A OFFSET PEDESTAL
#define ADS4222_REGC1        0xC1      // CH B OFFSET PEDESTAL
#define ADS4222_REGCF        0xCF      // FREEZE OFFSET CORR, OFFSET CORR TIME CONSTANT
#define ADS4222_REGDB        0xDB      // LOW SPEED MODE CH B
#define ADS4222_REGEF        0xEF      // EN LOW SPEED MODE
#define ADS4222_REGF1        0xF1      // EN LVDS SWING
#define ADS4222_REGF2        0xF2      // LOW SPEED MODE CH A

// Register 00 options (RESET, READOUT)
#define REG00_RESERVED_OFFSET               2                                       // B:[7:2] Reserved, must be 0
#define REG00_RESERVED_MASK                 (0b111111 << REG00_RESERVED_OFFSET)     // B:[7:2] Reserved, must be 0
#define REG00_RESET_MASK                    (0b1 << 1)                              // B:1 Software reset the device (resets all internal registers to the default values and self-clears to 0)
#define REG00_READOUT_MASK                  (0b1 << 0)                              // B:0 Device readout bit (set to 1 to enable reading and clear to 0 to end reading)

// Register 01 options (LVDS SWING)
#define REG01_LVDS_SWING_OFFSET             2                                       // B:[7:2] LVDS swing control (with external 100-Ω termination).
#define REG01_LVDS_SWING_MASK               (0b111111 << REG01_LVDS_SWING_OFFSET)   // B:[7:2] LVDS swing control. Set the EN LVDS SWING bit to 1 before programming swing.
#define REG01_LVDS_SWING_350_MASK           (0b000000 << REG01_LVDS_SWING_OFFSET)   // B:[7:2] ±350 mV (default)
#define REG01_LVDS_SWING_410_MASK           (0b011011 << REG01_LVDS_SWING_OFFSET)   // B:[7:2] ±410 mV
#define REG01_LVDS_SWING_465_MASK           (0b110010 << REG01_LVDS_SWING_OFFSET)   // B:[7:2] ±465 mV
#define REG01_LVDS_SWING_570_MASK           (0b010100 << REG01_LVDS_SWING_OFFSET)   // B:[7:2] ±570 mV
#define REG01_LVDS_SWING_200_MASK           (0b111110 << REG01_LVDS_SWING_OFFSET)   // B:[7:2] ±200 mV
#define REG01_LVDS_SWING_125_MASK           (0b001111 << REG01_LVDS_SWING_OFFSET)   // B:[7:2] ±125 mV
#define REG01_RESERVED_MASK                 0b11                                    // B:[1:0] Reserved, must be 0

// Register 03 options (HIGH PERF MODE)
#define REG03_RESERVED_OFFSET               2                                       // B:[7:2] Reserved, must be 0
#define REG03_RESERVED_MASK                 (0b111111 << REG03_RESERVED_OFFSET)     // B:[7:2] Reserved, must be 0
#define REG03_HIGH_PERF_MODE_MASK           0b11                                    // B:[1:0] High performance mode settings
#define REG03_HIGH_PERF_MODE_OFF_MASK       0b00                                    // B:[1:0] Default performance
#define REG03_HIGH_PERF_MODE_ON_MASK        0b11                                    // B:[1:0] High performance mode - obtain the best performance across sample clock and input signal frequencies

// Register 25 options (CH A GAIN, CH A TEST PATTERNS)
#define REG25_CH_A_GAIN_OFFSET              4                                       // B:[7:4] Channel A gain setting (steps of 0.5 dB from 0 (default) to 6 dB)
#define REG25_CH_A_GAIN_MASK                (0b1111 << REG25_CH_A_GAIN_OFFSET)      // B:[7:4] Channel A gain setting
#define REG25_CH_A_GAIN_0_0_MASK            (0b0000 << REG25_CH_A_GAIN_OFFSET)      // B:[7:4] 0 dB (default)
#define REG25_CH_A_GAIN_0_5_MASK            (0b0001 << REG25_CH_A_GAIN_OFFSET)      // B:[7:4] 0.5 dB
#define REG25_CH_A_GAIN_1_0_MASK            (0b0010 << REG25_CH_A_GAIN_OFFSET)      // B:[7:4] 1 dB
#define REG25_CH_A_GAIN_1_5_MASK            (0b0011 << REG25_CH_A_GAIN_OFFSET)      // B:[7:4] 1.5 dB
#define REG25_CH_A_GAIN_2_0_MASK            (0b0100 << REG25_CH_A_GAIN_OFFSET)      // B:[7:4] 2 dB
#define REG25_CH_A_GAIN_2_5_MASK            (0b0101 << REG25_CH_A_GAIN_OFFSET)      // B:[7:4] 2.5 dB
#define REG25_CH_A_GAIN_3_0_MASK            (0b0110 << REG25_CH_A_GAIN_OFFSET)      // B:[7:4] 3 dB
#define REG25_CH_A_GAIN_3_5_MASK            (0b0111 << REG25_CH_A_GAIN_OFFSET)      // B:[7:4] 3.5 dB
#define REG25_CH_A_GAIN_4_0_MASK            (0b1000 << REG25_CH_A_GAIN_OFFSET)      // B:[7:4] 4 dB
#define REG25_CH_A_GAIN_4_5_MASK            (0b1001 << REG25_CH_A_GAIN_OFFSET)      // B:[7:4] 4.5 dB
#define REG25_CH_A_GAIN_5_0_MASK            (0b1010 << REG25_CH_A_GAIN_OFFSET)      // B:[7:4] 5 dB
#define REG25_CH_A_GAIN_5_5_MASK            (0b1011 << REG25_CH_A_GAIN_OFFSET)      // B:[7:4] 5.5 dB
#define REG25_CH_A_GAIN_6_0_MASK            (0b1100 << REG25_CH_A_GAIN_OFFSET)      // B:[7:4] 6 dB
#define REG25_RESERVED_MASK                 (0b1 << 3)                              // B:3 Reserved, must be 0
#define REG25_CH_A_TEST_PATTERNS_MASK       0b111                                   // B:[2:0] Channel A test patterns
#define REG25_CH_A_TEST_PATTERNS_OFF_MASK   0b000                                   // B:[2:0] Test patterns off (default)
#define REG25_CH_A_TEST_PATTERNS_ZEROS_MASK 0b001                                   // B:[2:0] All zeros on output A
#define REG25_CH_A_TEST_PATTERNS_ONES_MASK  0b010                                   // B:[2:0] All ones on output A
#define REG25_CH_A_TEST_PATTERNS_ALT_MASK   0b011                                   // B:[2:0] Alternate 10...10 and 01...01 on output A on each clock cycle
#define REG25_CH_A_TEST_PATTERNS_RAMP_MASK  0b100                                   // B:[2:0] Ramp pattern on output A every 4 clock cycles
#define REG25_CH_A_TEST_PATTERNS_CUST_MASK  0b101                                   // B:[2:0] Custom pattern D on output A

// Register 29 options (DATA FORMAT)
#define REG29_RESERVED_MASK                 0b11100111                              // B:[7:5,2:0] Reserved, must be 0
#define REG29_DATA_FORMAT_OFFSET            3                                       // B:[4:3] Data format selection
#define REG29_DATA_FORMAT_MASK              (0b11 << REG29_DATA_FORMAT_OFFSET)      // B:[4:3] Data format selection
#define REG29_DATA_FORMAT_TWOS_COMPL_MASK   0b00                                    // B:[4:3] Two's complement (default)
#define REG29_DATA_FORMAT_OFFSET_BIN_MASK   0b11                                    // B:[4:5] Offset binary

// Register 2B options (CH B GAIN, CH B TEST PATTERNS)
#define REG2B_CH_B_GAIN_OFFSET              4                                       // B:[7:4] Channel B gain setting (steps of 0.5 dB from 0 (default) to 6 dB)
#define REG2B_CH_B_GAIN_MASK                (0b1111 << REG2B_CH_B_GAIN_OFFSET)      // B:[7:4] Channel B gain setting
#define REG2B_CH_B_GAIN_0_0_MASK            (0b0000 << REG2B_CH_B_GAIN_OFFSET)      // B:[7:4] 0 dB (default)
#define REG2B_CH_B_GAIN_0_5_MASK            (0b0001 << REG2B_CH_B_GAIN_OFFSET)      // B:[7:4] 0.5 dB
#define REG2B_CH_B_GAIN_1_0_MASK            (0b0010 << REG2B_CH_B_GAIN_OFFSET)      // B:[7:4] 1 dB
#define REG2B_CH_B_GAIN_1_5_MASK            (0b0011 << REG2B_CH_B_GAIN_OFFSET)      // B:[7:4] 1.5 dB
#define REG2B_CH_B_GAIN_2_0_MASK            (0b0100 << REG2B_CH_B_GAIN_OFFSET)      // B:[7:4] 2 dB
#define REG2B_CH_B_GAIN_2_5_MASK            (0b0101 << REG2B_CH_B_GAIN_OFFSET)      // B:[7:4] 2.5 dB
#define REG2B_CH_B_GAIN_3_0_MASK            (0b0110 << REG2B_CH_B_GAIN_OFFSET)      // B:[7:4] 3 dB
#define REG2B_CH_B_GAIN_3_5_MASK            (0b0111 << REG2B_CH_B_GAIN_OFFSET)      // B:[7:4] 3.5 dB
#define REG2B_CH_B_GAIN_4_0_MASK            (0b1000 << REG2B_CH_B_GAIN_OFFSET)      // B:[7:4] 4 dB
#define REG2B_CH_B_GAIN_4_5_MASK            (0b1001 << REG2B_CH_B_GAIN_OFFSET)      // B:[7:4] 4.5 dB
#define REG2B_CH_B_GAIN_5_0_MASK            (0b1010 << REG2B_CH_B_GAIN_OFFSET)      // B:[7:4] 5 dB
#define REG2B_CH_B_GAIN_5_5_MASK            (0b1011 << REG2B_CH_B_GAIN_OFFSET)      // B:[7:4] 5.5 dB
#define REG2B_CH_B_GAIN_6_0_MASK            (0b1100 << REG2B_CH_B_GAIN_OFFSET)      // B:[7:4] 6 dB
#define REG2B_RESERVED_MASK                 (0b1 << 3)                              // B:3 Reserved, must be 0
#define REG2B_CH_B_TEST_PATTERNS_MASK       0b111                                   // B:[2:0] Channel B test patterns
#define REG2B_CH_B_TEST_PATTERNS_OFF_MASK   0b000                                   // B:[2:0] Test patterns off (default)
#define REG2B_CH_B_TEST_PATTERNS_ZEROS_MASK 0b001                                   // B:[2:0] All zeros on output B
#define REG2B_CH_B_TEST_PATTERNS_ONES_MASK  0b010                                   // B:[2:0] All ones on output B
#define REG2B_CH_B_TEST_PATTERNS_ALT_MASK   0b011                                   // B:[2:0] Alternate 10...10 and 01...01 on output B on each clock cycle
#define REG2B_CH_B_TEST_PATTERNS_RAMP_MASK  0b100                                   // B:[2:0] Ramp pattern on output B every 4 clock cycles
#define REG2B_CH_B_TEST_PATTERNS_CUST_MASK  0b101                                   // B:[2:0] Custom pattern D on output B

// Register 3D options (ENABLE OFFSET CORR)
#define REG3D_RESERVED_MASK                 0b11011111                              // B:[7:6,4:0] Reserved, must be 0
#define REG3D_ENABLE_OFFSET_CORR_MASK       (0b1 << 5)                              // B:5 Enable offset correction (1 = on, 0 = off)

// Register 3F options (CUSTOM PATTERN D[13:8])
#define REG3F_RESERVED_OFFSET               6                                       // B:[7:6] Reserved, must be 0
#define REG3F_RESERVED_MASK                 (0b11 << REG3F_RESERVED_OFFSET)         // B:[7:6] Reserved, must be 0
#define REG3F_CUSTOM_PATTERN_MASK           0b111111                                // B:[5:0] Custom pattern D[13:8] (ADS424x uses all bits, ADS422x ignore top 2 bits)

#define CUSTOM_PATTERN_TOP_OFFSET           8                                       // Offset for extracting D[13:8] from int
#define CUSTOM_PATTERN_TOP_MASK             (0b111111 << CUSTOM_PATTERN_TOP_OFFSET) // Mask for extracting D[13:8] from int

// Register 40 options (CUSTOM PATTERN D[7:0])
#define REG40_CUSTOM_PATTERN_MASK           0x11111111                              // B:[7:0] Custom pattern D[7:0]

#define CUSTOM_PATTERN_BOT_MASK             0b11111111                              // Mask for extracting D[7:0] from int

// Register 41 options (LVDS CMOS, CMOS CLKOUT STRENGTH, DIS OBUF)
#define REG41_LVDS_CMOS_OFFSET              6                                       // B:[7:6] LVDS CMOS output mode
#define REG41_LVDS_CMOS_MASK                (0b11 << REG41_LVDS_CMOS_OFFSET)        // B:[7:6] LVDS CMOS output mode
#define REG41_LVDS_CMOS_LVDS_MASK           (0b00 << REG41_LVDS_CMOS_OFFSET)        // B:[7:6] DDR LVDS output mode
#define REG41_LVDS_CMOS_CMOS_MASK           (0b11 << REG41_LVDS_CMOS_OFFSET)        // B:[7:6] CMOS output mode
#define REG41_CMOS_CLKOUT_STRE_OFFSET       4                                       // B:[5:4] CMOS CLKOUT strength
#define REG41_CMOS_CLKOUT_STRENGTH_MASK     (0b11 << REG41_CMOS_CLKOUT_STRE_OFFSET) // B:[5:4] CMOS CLKOUT strength
#define REG41_CMOS_CLKOUT_STRENGTH_MAX_MASK 0b00                                    // B:[5:4] Maximum drive strength
#define REG41_CMOS_CLKOUT_STRENGTH_MED_MASK 0b01                                    // B:[5:4] Medium drive strength
#define REG41_CMOS_CLKOUT_STRENGTH_LOW_MASK 0b10                                    // B:[5:4] Low drive strength
#define REG41_CMOS_CLKOUT_STRENGTH_MIN_MASK 0b11                                    // B:[5:4] Minimum/very low drive strength
#define REG41_RESERVED_OFFSET               2                                       // B:[3:2] Reserved, must be 0
#define REG41_RESERVED_MASK                 (0b11 << REG41_RESERVED_OFFSET)         // B:[3:2] Reserved, must be 0
#define REG41_DIS_OBUF_MASK                 0b11                                    // B:[1:0] Output buffers disable mask
#define REG41_DIS_OBUF_DEF_MASK             0b00                                    // B:[1:0] Output buffers enabled (default)
#define REG41_DIS_OBUF_B_MASK               0b01                                    // B:[1:0] Output buffers disabled for channel B
#define REG41_DIS_OBUF_A_MASK               0b10                                    // B:[1:0] Output buffers disabled for channel A
#define REG41_DIS_OBUF_ALL_MASK            0b11                                     // B:[1:0] Output buffers disabled for both channels and the output clock

// Register 42 options (CLKOUT FALL POSN, CLKOUT RISE POSN, EN DIGITAL)
#define REG42_CLKOUT_FALL_POSN_OFFSET       6                                       // B:[7:6] CLKOUT falling edge position
#define REG42_CLKOUT_FALL_POSN_MASK         (0b11 << REG42_CLKOUT_FALL_POSN_OFFSET) // B:[7:6] CLKOUT falling edge position (- advance, + delay)
#define REG42_CLKOUT_FALL_POSN_DEF_MASK     0b00                                    // B:[7:6] Default
#define REG42_CLKOUT_FALL_POSN_01_MASK      0b01                                    // B:[7:6] LVDS: -450 ps, CMOS: +150 ps
#define REG42_CLKOUT_FALL_POSN_10_MASK      0b10                                    // B:[7:6] LVDS: -150 ps, CMOS: Do not use
#define REG42_CLKOUT_FALL_POSN_11_MASK      0b11                                    // B:[7:6] LVDS: +550 ps, CMOS: -100 ps
#define REG42_CLKOUT_RISE_POSN_OFFSET       4                                       // B:[5:4] CLKOUT rising edge position
#define REG42_CLKOUT_RISE_POSN_MASK         (0b11 << REG42_CLKOUT_RISE_POSN_OFFSET) // B:[5:4] CLKOUT rising edge position (- advance, + delay)
#define REG42_CLKOUT_RISE_POSN_DEF_MASK     0b00                                    // B:[5:4] Default
#define REG42_CLKOUT_RISE_POSN_01_MASK      0b01                                    // B:[5:4] LVDS: -450 ps, CMOS: +150 ps
#define REG42_CLKOUT_RISE_POSN_10_MASK      0b10                                    // B:[5:4] LVDS: -150 ps, CMOS: Do not use
#define REG42_CLKOUT_RISE_POSN_11_MASK      0b11                                    // B:[5:4] LVDS: +550 ps, CMOS: -100 ps
#define REG42_EN_DIGITAL_MASK               (0b1 << 3)                              // B:3 Enable digital functions (test patterns, gain, offset correction)
#define REG42_RESERVED_MASK                 0b111                                   // B:[2:0] Reserved, must be 0

// Register 45 options (STBY, LVDS CLKOUT STRENGTH, LVDS DATA STRENGTH, PDN GLOBAL)
#define REG45_RESERVED_MASK                 0b11011                                 // B:[4:3,1:0] Reserved, must be 0
#define REG45_STBY_MASK                     (0b1 << 7)                              // B:7 Standby mode (1 = both channels in standby, 0 = normal operation (wakeup time ~50us))
#define REG45_LVDS_CLKOUT_STRENGTH_MASK     (0b1 << 6)                              // B:6 LVDS CLKOUT strength (0 = default 100 Ω, 1 = 50 Ω)
#define REG45_LVDS_DATA_STRENGTH_MASK       (0b1 << 5)                              // B:5 LVDS data strength (0 = default 100 Ω, 1 = 50 Ω)
#define REG45_PDN_GLOBAL_MASK               (0b1 << 2)                              // B:2 Global power down (0 = normal operation, 1 = Total power down; all ADC channels, internal references, and output buffers are powered down. Wakeup time from this mode is slow ~100 μs)

// Register 4A options (HIGH FREQ MODE CH B - improves SFDR on high frequencies: f_in > 200 MHz)
#define REG4A_RESERVED_OFFSET               1                                       // B:[7:1] Reserved, must be 0
#define REG4A_RESERVED_MASK                 (0b1111111 << REG4A_RESERVED_OFFSET)    // B:[7:1] Reserved, must be 0
#define REG4A_HIGH_FREQ_MODE_CH_B_MASK      0b1                                     // B:0 High frequency mode for channel B (0 = default, 1 = improves SFDR on high frequencies: f_in > 200 MHz)

// Register 58 options (HIGH FREQ MODE CH A)
#define REG58_RESERVED_OFFSET               1                                       // B:[7:1] Reserved, must be 0
#define REG58_RESERVED_MASK                 (0b1111111 << REG58_RESERVED_OFFSET)    // B:[7:1] Reserved, must be 0
#define REG58_HIGH_FREQ_MODE_CH_A_MASK      0b1                                     // B:0 High frequency mode for channel A (0 = default, 1 = improves SFDR on high frequencies: f_in > 200 MHz)

// Register BF options (CH A OFFSET PEDESTAL)
#define REGBF_CH_A_OFFSET_PEDESTAL_OFFSET   2                                       // B:[7:2] Channel A offset pedestal (refer to the data sheet SBAS533E page 76 for more information)
#define REGBF_CH_A_OFFSET_PEDESTAL_MASK     (0b111111 << REGBF_CH_A_OFFSET_PEDESTAL_OFFSET) // B:[7:2] Channel A offset pedestal
#define REGBF_RESERVED_MASK                 0b11                                    // B:[1:0] Reserved, must be 0

// Register C1 options (CH B OFFSET PEDESTAL)
#define REGC1_CH_B_OFFSET_PEDESTAL_OFFSET   2                                       // B:[7:2] Channel B offset pedestal (refer to the data sheet SBAS533E page 77 for more information)
#define REGC1_CH_B_OFFSET_PEDESTAL_MASK     (0b111111 << REGC1_CH_B_OFFSET_PEDESTAL_OFFSET) // B:[7:2] Channel B offset pedestal
#define REGC1_RESERVED_MASK                 0b11                                    // B:[1:0] Reserved, must be 0

// Register CF options (FREEZE OFFSET CORR, OFFSET CORR TIME CONSTANT)
#define REGCF_RESERVED_MASK                 0b1000011                               // B:[6,1:0] Reserved, must be 0
#define REGCF_FREEZE_OFFSET_CORR_MASK       (0b1 << 7)                              // B:7 Freeze offset correction if EN OFFSET CORR is set (1 = frozen, 0 = not frozen)
#define REGCF_OFFSET_CORR_TIME_CONST_OFFSET 2                                       // B:[5:2] Offset correction time constant (refer to the data sheet SBAS533E page 58 for more information)
#define REGCF_OFFSET_CORR_TIME_CONST_MASK       (0b1111 << REGCF_OFFSET_CORR_TIME_CONST_OFFSET)     // B:[5:2] Offset correction time constant
#define REGCF_OFFSET_CORR_TIME_CONST_1M_MASK    (0b0000 << REGCF_OFFSET_CORR_TIME_CONST_OFFSET)     // B:[5:2] 1M clock cycles
#define REGCF_OFFSET_CORR_TIME_CONST_2M_MASK    (0b0001 << REGCF_OFFSET_CORR_TIME_CONST_OFFSET)     // B:[5:2] 2M clock cycles
#define REGCF_OFFSET_CORR_TIME_CONST_4M_MASK    (0b0010 << REGCF_OFFSET_CORR_TIME_CONST_OFFSET)     // B:[5:2] 4M clock cycles
#define REGCF_OFFSET_CORR_TIME_CONST_8M_MASK    (0b0011 << REGCF_OFFSET_CORR_TIME_CONST_OFFSET)     // B:[5:2] 8M clock cycles
#define REGCF_OFFSET_CORR_TIME_CONST_16M_MASK   (0b0100 << REGCF_OFFSET_CORR_TIME_CONST_OFFSET)     // B:[5:2] 16M clock cycles
#define REGCF_OFFSET_CORR_TIME_CONST_32M_MASK   (0b0101 << REGCF_OFFSET_CORR_TIME_CONST_OFFSET)     // B:[5:2] 32M clock cycles
#define REGCF_OFFSET_CORR_TIME_CONST_64M_MASK   (0b0110 << REGCF_OFFSET_CORR_TIME_CONST_OFFSET)     // B:[5:2] 64M clock cycles
#define REGCF_OFFSET_CORR_TIME_CONST_128M_MASK  (0b0111 << REGCF_OFFSET_CORR_TIME_CONST_OFFSET)     // B:[5:2] 128M clock cycles
#define REGCF_OFFSET_CORR_TIME_CONST_256M_MASK  (0b1000 << REGCF_OFFSET_CORR_TIME_CONST_OFFSET)     // B:[5:2] 256M clock cycles
#define REGCF_OFFSET_CORR_TIME_CONST_512M_MASK  (0b1001 << REGCF_OFFSET_CORR_TIME_CONST_OFFSET)     // B:[5:2] 512M clock cycles
#define REGCF_OFFSET_CORR_TIME_CONST_1G_MASK    (0b1010 << REGCF_OFFSET_CORR_TIME_CONST_OFFSET)     // B:[5:2] 1G clock cycles
#define REGCF_OFFSET_CORR_TIME_CONST_2G_MASK    (0b1011 << REGCF_OFFSET_CORR_TIME_CONST_OFFSET)     // B:[5:2] 2G clock cycles

// Register DB options (LOW SPEED MODE CH B - not applicable to ADS4242 and ADS4222)
#define REGDB_RESERVED_OFFSET               1                                       // B:[7:1] Reserved, must be 0
#define REGDB_RESERVED_MASK                 (0b1111111 << REGDB_RESERVED_OFFSET)    // B:[7:1] Reserved, must be 0
#define REGDB_LOW_SPEED_MODE_CH_B_MASK      0b1                                     // B:0 Low speed mode for channel B (0 = off, 1 = on)

// Register EF options (EN LOW SPEED MODE - not applicable to ADS4242 and ADS4222)
#define REGEF_RESERVED_MASK                 0b11101111                              // B:[7:5,3:0] Reserved, must be 0
#define REGEF_EN_LOW_SPEED_MODE_MASK        (0b1 << 4)                              // B:4 Enable low speed mode (0 = off, 1 = on - further settings in register DB and F2)

// Register F1 options (EN LVDS SWING)
#define REGF1_RESERVED_MASK                 0b11111100                              // B:[7:2] Reserved, must be 0
#define REGF1_EN_LVDS_SWING_MASK            0b11                                    // B:[1:0] Enable LVDS swing (0 = off, 1 = on)
#define REGF1_EN_LVDS_SWING_OFF_MASK        0b00                                    // B:[1:0] LVDS swing control off (in the LVDS SWING register)
#define REGF1_EN_LVDS_SWING_ON_MASK         0b11                                    // B:[1:0] LVDS swing control on (in the LVDS SWING register)

// Register F2 options (LOW SPEED MODE CH A - not applicable to ADS4242 and ADS4222)
#define REGF2_RESERVED_MASK                 0b11110111                              // B:[7:4,2:0] Reserved, must be 0
#define REGF2_LOW_SPEED_MODE_CH_A_MASK      (0b1 << 3)                              // B:3 Low speed mode for channel A (0 = off, 1 = on)

// This needs to be visible from C code
#ifdef __cplusplus
extern "C" {
#endif

// Function prototypes
void send_ADS4222(uint16_t addr, uint16_t data);
uint16_t read_ADS4222(uint16_t addr);

void init_ADS4222();

#ifdef __cplusplus
}
#endif

#endif //VNA_ADS4222_H
