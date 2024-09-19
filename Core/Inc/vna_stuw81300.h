#ifndef STUW81300_H_INCLUDED
#define STUW81300_H_INCLUDED

#include "main.h"
#include "types.h"
#include "vna_tx_utils.h"

// This driver is for the ST STuW81300 Wide-band Microwave Frac-Integer-N Integrated Synthesizer.
// Product page: https://www.st.com/en/wireless-connectivity/stuw81300.html
// Datasheet: https://www.st.com/resource/en/datasheet/stuw81300.pdf

#define PLL_REF_FREQ            60000000    // 60 MHz

#define MIN_FREQ                8000000000  // 8 GHz
#define MAX_FREQ                16000000000 // 16 GHz
#define FREQ_RANGE              MIN_FREQ, MAX_FREQ

#define USE_DOUBLE_BUFFERING                // define to use double buffering

// only uncomment one of the following lines
//#define CORE_VCC_3V3
#define CORE_VCC_4V5

// SPI settings (use SPI (hspi2 on H5))
#define PLL_SPI         hspi2
#define PLL_LE_PORT     SPI2_RST_GPIO_Port
#define PLL_LE_PIN      SPI2_RST_Pin

#define PLL_SPI_DATA_SIZE   1 // in chunks defined in the HAL SPI library
#define PLL_SPI_DELAY       50 // in us, should be enough
#define PLL_SPI_TIMEOUT     10 // in ms, should be enough

// Register component shifts and masks
#define STUW81300_RW_MASK           0x80000000                                      // R/W bit (1 = Read, 0 = Write)
#define STUW81300_ADDR_MASK         0x0F                                            // Address bits before shifting (bits 3 to 0)
#define STUW81300_ADDR_SHIFT        27                                              // Shift for address bits
#define STUW81300_ADDR_MASK_SH      (STUW81300_ADDR_MASK << STUW81300_ADDR_SHIFT)   // Address bits (bits 30 to 27)
#define STUW81300_DATA_MASK         0x07FFFFFF                                      // Data bits (bits 26 to 0)

// Register addresses
#define STUW81300_REG0      0x00
#define STUW81300_REG1      0x01
#define STUW81300_REG2      0x02
#define STUW81300_REG3      0x03
#define STUW81300_REG4      0x04
#define STUW81300_REG5      0x05
#define STUW81300_REG6      0x06
#define STUW81300_REG7      0x07
#define STUW81300_REG8      0x08
#define STUW81300_REG9      0x09
#define STUW81300_REG10     0x0A
#define STUW81300_REG11     0x0B

// Register 0 options (Master register. N divider, CP current. Writing to this register starts a VCO calibration)
#define REG0_VCO_CALB_DISABLE_MASK          (0b1 << 26)                             // B:26         Disable VCO calibration (should not be used - this bit is always 0 and is write only)
#define REG0_CP_SEL_OFFSET                  21                                      // B:[25:21]    Charge pump pulse current 0 to 4.9 mA; step~158μA 00000 to 11111
#define REG0_CP_SEL_MASK                    (0x1F << REG0_CP_SEL_OFFSET)            // B:[25:21]    Charge pump pulse current mask
#define REG0_PFD_DEL_OFFSET                 19                                      // B:[20:19]    PFD delay offset (anti-backlash)
#define REG0_PFD_DEL_0                      (0b00 << PFD_DEL_OFFSET)                // B:[20:19]    1.2 ns / 0 A (default)
#define REG0_PFD_DEL_1                      (0b01 << PFD_DEL_OFFSET)                // B:[20:19]    1.9 ns / 0.25*ICP
#define REG0_PFD_DEL_2                      (0b10 << PFD_DEL_OFFSET)                // B:[20:19]    2.5 ns / 0.5*ICP
#define REG0_PFD_DEL_3                      (0b11 << PFD_DEL_OFFSET)                // B:[20:19]    3.0 ns / 0.75*ICP
#define REG0_RESERVED_MASK                  (0b11 << 17)                            // B:[18:17]    Reserved, leave at 00
#define REG0_N_INT_OFFSET                   0                                       // B:[16:0]     Integer part of N divider
#define REG0_N_INT_MASK                     0x1FFFF                                 // B:[16:0]     Integer part of N divider

#define REG0_N_INT_MIN                      1                                       // Minimum N_INT value
#define REG0_N_INT_MAX                      131071                                  // Maximum N_INT value
#define REG0_N_INT_RANGE                    REG0_N_INT_MIN, REG0_N_INT_MAX          // To be used as the second pair of parameters in in_range()
#define REG0_N_INT_FRAC_EDGE                512                                     // If N_INT is ≥ 512, N_FRAC and MOD are ignored

// Register 1 options (FRAC value, RF1 output control)
#define REG1_DBR_MASK                       (0b1 << 26)                             // B:26         Double buffer enable (only transfer after DB0 is written)
#define REG1_RESERVED_MASK                  (0b1 << 25)                             // B:25         Reserved, leave at 0
#define REG1_RF1_OUT_PD_MASK                (0b1 << 24)                             // B:24         RF1 output power down (0 = on, 1 = off)
#define REG1_MAN_CALB_EN_MASK               (0b1 << 23)                             // B:23         Manual calibration enable (0 = auto [VCO_SEL, VCO_WORD are ignored], 1 = manual)
#define REG1_PLL_SEL_MASK                   (0b1 << 22)                             // B:22         PLL selection (0 = VCO direct to PLL, 1 = VCO/2 to PLL -> 2x frequency)
#define REG1_RF1_SEL_MASK                   (0b1 << 21)                             // B:21         RF1 output selection (0 = VCO, 1 = VCO/2)
#define REG1_N_FRAC_OFFSET                  0                                       // B:[20:0]     Fractional part of N divider
#define REG1_N_FRAC_MASK                    0x1FFFFF                                // B:[20:0]     Fractional part of N divider - 0 to 2097151 (must be < MOD)

#define REG1_N_FRAC_FREQ_EDGE               6000000000                              // If Fvco is ≥ 6 GHz we need to use a frequency divider

#define REG1_N_FRAC_MIN                     0                                       // Minimum FRAC value
#define REG1_N_FRAC_MAX                     2097151                                 // Maximum FRAC value
#define REG1_N_FRAC_RANGE                   REG1_N_FRAC_MIN, REG1_N_FRAC_MAX        // To be used as the second pair of parameters in in_range()

// Register 2 options (MOD value, RF2 output control)
#define REG2_DBR_MASK                       (0b1 << 26)                             // B:26         Double buffer enable (only transfer after DB0 is written)
#define REG2_DSM_CLK_DISABLE_MASK           (0b1 << 25)                             // B:25         For test purposes (must be always 0)
#define REG2_RESERVED_OFFSET                22                                      // B:[24:22]    Reserved, leave at 000
#define REG2_RESERVED_MASK                  (0b111 << REG2_RESERVED_OFFSET)         // B:[24:22]    Reserved, leave at 000
#define REG2_RF2_OUT_PD_MASK                (0b1 << 21)                             // B:21         RF2 output power down (0 = on [only if RF1 is powered down], 1 = off)
#define REG2_MOD_OFFSET                     0                                       // B:[20:0]     Modulus value
#define REG2_MOD_MASK                       0x1FFFFF                                // B:[20:0]     Modulus value - 2 to 2097151

#define REG2_MOD_MIN                        2                                       // Minimum MOD value
#define REG2_MOD_MAX                        2097151                                 // Maximum MOD value
#define REG2_MOD_RANGE                      REG2_MOD_MIN, REG2_MOD_MAX              // To be used as the second pair of parameters in in_range()

// Register 3 options (R divider, CP leakage, CP down-split pulse, Ref. Path selection, Device power down)
#define REG3_DBR_MASK                           (0b1 << 26)                         // B:26        Double buffer enable (only transfer after DB0 is written)
#define REG3_PD_MASK                            (0b1 << 25)                         // B:25        Power down all blocks except LDOs (0 = on, 1 = off)
#define REG3_CP_LEAK_x2_MASK                    (0b1 << 24)                         // B:24        Charge pump leakage current x2 (0 = 10 μA step, 1 = 20 μA step)
#define REG3_CP_LEAK_OFFSET                     19                                  // B:[23:19]   Charge pump leakage current bit offset
#define REG3_CP_LEAK_MASK                       (0x1F << REG3_CP_LEAK_OFFSET)       // B:[23:19]   Charge pump leakage current - 0 to 31 (0 to 310/620 μA depending on CP_SEL)
#define REG3_CP_LEAK_DIR_MASK                   (0b1 << 18)                         // B:18        Charge pump leakage current direction (0 = down-leakage [current sink], 1 = up-leakage [current source])
#define REG3_DNSPLIT_EN_MASK                    (0b1 << 17)                         // B:17        Charge pump down-split pulse enable set by PFD_DEL[1:0] (0 = off, 1 = on)
#define REG3_PFD_DEL_MODE_OFFSET                15                                  // B:[16:15]   PFD delay mode offset values set by PFD_DEL[1:0]
#define REG3_PFD_DEL_MODE_NO_DELAY_MASK         (0b00 << REG3_PFD_DEL_MODE_OFFSET)  // B:[16:15]   PFD delay mode - no delay (default)
#define REG3_PFD_DEL_MODE_VCO_DIV_DELAYED_MASK  (0b01 << REG3_PFD_DEL_MODE_OFFSET)  // B:[16:15]   PFD delay mode - VCO delayed
#define REG3_PFD_DEL_MODE_REF_DIV_DELAYED_MASK  (0b10 << REG3_PFD_DEL_MODE_OFFSET)  // B:[16:15]   PFD delay mode - Ref. delayed
#define REG3_PFD_DEL_MODE_RESERVED_MASK         (0b11 << REG3_PFD_DEL_MODE_OFFSET)  // B:[16:15]   PFD delay mode - reserved, should not be used
#define REG3_REF_PATH_SEL_OFFSET                13                                  // B:[14:13]   Reference clock path selection offset
#define REG3_REF_PATH_SEL_MASK                  (0b00 << REG3_REF_PATH_SEL_OFFSET)  // B:[14:13]   Direct path
#define REG3_REF_PATH_SEL_x2_MASK               (0b01 << REG3_REF_PATH_SEL_OFFSET)  // B:[14:13]   Doubled in single mode; Not Applicable in differential mode
#define REG3_REF_PATH_SEL_DIV2_MASK             (0b10 << REG3_REF_PATH_SEL_OFFSET)  // B:[14:13]   Divided by 2
#define REG3_REF_PATH_SEL_DIV4_MASK             (0b11 << REG3_REF_PATH_SEL_OFFSET)  // B:[14:13]   Divided by 4
#define REG3_R_DIV_OFFSET                       0                                   // B:[12:0]    R divider value offset
#define REG3_R_DIV_MASK                         0x1FFF                              // B:[12:0]    Reference clock divider ratio - 1 to 8191

#define REG3_R_DIV_MIN                          1                                   // Minimum R_DIV value
#define REG3_R_DIV_MAX                          8191                                // Maximum R_DIV value
#define REG3_R_DIV_RANGE                        REG3_R_DIV_MIN, REG3_R_DIV_MAX      // To be used as the second pair of parameters in in_range()

// Register 4 options (Lock det. control, Ref. Buffer, CP supply mode, VCO settings, Output power control)
// [26:25], [22:20], [18] and [13] are reserved and should be left at 0
#define REG4_RESERVED_OFFSET                13                                      // Offset for all the reserved bits
#define REG4_RESERVED_MASK                  (0x33A1 << REG4_RESERVED_OFFSET)        // B:[26:25,22:20,18,13]    Mask for all the reserved bits - should be left at 0
#define REG4_CALB_3V3_MODE1_MASK            (0b1 << 24)                             // B:24         calibrator supply mode bit1 - 0 when VCC_VCO_Core = 4.5 V and 1 when VCC_VCO_Core = 3.3 V
#define REG4_RF_OUT_3V3_MASK                (0b1 << 23)                             // B:23         0 when VCC_RFOUT = 4.5 V and 1 when VCC_RFOUT = 3.3 V
#define REG4_EXT_VCO_EN_MASK                (0b1 << 19)                             // B:19         External VCO enable (0 = ext. VCO buffer disabled; integrated VCOs are used, 1 = ext. VCO buffer enabled; external VCO required)
#define REG4_VCO_AMP_OFFSET                 15                                      // B:[17:15]    VCO amplitude control offset
#define REG4_VCO_AMP_MASK                   (0b111 << REG4_VCO_AMP_OFFSET)          // B:[17:15]    VCO amplitude control (000 to 010 when VCC_VCO_Core = 3.3 V, 000 to 111 when VCC_VCO_Core = 4.5 V)
#define REG4_CALB_3V3_MODE0_MASK            (0b1 << 14)                             // B:14         calibrator supply mode bit0 - 0 when VCC_VCO_Core = 4.5 V and 1 when VCC_VCO_Core = 3.3 V
#define REG4_VCALB_MODE_MASK                (0b1 << 12)                             // B:12         VCO calibrator mode (0 = when F_VCO ≤ 4500 MHz, 1 = when F_VCO > 4500 MHz or VCC_VCO_Core = 3.3 V)
#define REG4_KVCO_COMP_DIS_MASK             (0b1 << 11)                             // B:11         VCO compensation disable (0 = default - CP current auto-adjusted to compensate KVCO variation, 1 = CP current fixed by CP_SEL settings)
#define REG4_PFD_POL_MASK                   (0b1 << 10)                             // B:10         PFD polarity (0 = standard mode (default), 1 = “inverted” mode (to be used only with active inverting loop filter or with VCO with negative tuning characteristics))
#define REG4_REF_BUFF_MODE_OFFSET           8                                       // B:[9:8]      Reference clock buffer mode offset
#define REG4_REF_BUFF_MODE_DIFF_MASK        (0b01 << REG4_REF_BUFF_MODE_OFFSET)     // B:[9:8]      Differential mode (reference clock signal on pin #20 and #21)
#define REG4_REF_BUFF_MODE_XTAL_MASK        (0b10 << REG4_REF_BUFF_MODE_OFFSET)     // B:[9:8]      Crystal mode (Xtal oscillator enabled with crystal connected on pin #20 and #21)
#define REG4_REF_BUFF_MODE_SINGLE_MASK      (0b11 << REG4_REF_BUFF_MODE_OFFSET)     // B:[9:8]      Single ended mode (reference clock signal on pin #21)
#define REG4_MUTE_LOCK_EN_MASK              (0b1 << 7)                              // B:7          Mute on unlock enable (RF output stages are put OFF when PLL is unlocked) - 0 = off, 1 = on
#define REG4_LD_ACTIVELOW_MASK              (0b1 << 6)                              // B:6          Lock indicator polarity (0 = active high (LD=0 - unlocked, LD=1 - locked), 1 = active low (LD=1 - unlocked, LD=0 - locked))
#define REG4_LD_PREC_OFFSET                 3                                       // B:[5:3]      Lock detector precision offset
#define REG4_LD_PREC_MASK                   (0b111 << REG4_LD_PREC_OFFSET)          // B:[5:3]      Lock detector precision (0 = 2 ns (default for integer mode), 1 = 4 ns (default for fractional mode), 2 = 6 ns, 3 = 8 ns, 4 = 10 ns, 5 = 12 ns, 6 = 14 ns, 7 = 16 ns)
#define REG4_LD_COUNT_OFFSET                0                                       // B:[2:0]      Lock detector counter for lock condition offset
#define REG4_LD_COUNT_MASK                  0b111                                   // B:[2:0]      Lock detector counter for lock condition - (0=4, 1=8 (default for FPFD ~1 MHz in INT mode), 2=16, 3=64, 4=256, 5=1024 (default for F PFD ~50 MHz in FRAC/INT mode), 6=2048, 7=4096)

#define REG4_VCALB_MODE_EDGE                4500000000                              // Edge frequency for VCO calibrator mode

// Register 5 options (Low power mode control bit)
#define REG5_RESERVED_OFFSET                1                                       // B:[26:5]     Reserved, leave at 0
#define REG5_RESERVED_MASK                  (0x3FFFFF5 << REG5_RESERVED_OFFSET)     // B:[26:5]     Reserved, leave at 0
#define REG5_RF2_OUTBUF_LP_MASK             (0b1 << 4)                              // B:4          RF2 output buffer low power mode (0 = full power, 1 = low power)
#define REG5_DEMUX_LP_MASK                  (0b1 << 2)                              // B:2          RF demux low power mode (0 = full power, 1 = low power)
#define REG5_REF_BUFF_LP_MASK               (0b1 << 0)                              // B:0          Reference clock buffer low power mode (0 = full power, 1 = low power)

// Register 6 options (VCO Calibrator, Manual VCO control, DSM settings)
#define REG6_DITHERING_MASK                 (0b1 << 26)                             // B:26         Enables dithering of DSM output sequence (0 = off, 1 = on)
#define REG6_CP_UP_OFF_MASK                 (0b1 << 25)                             // B:25         For test purposes only; must be set to ‘0’
#define REG6_CP_DN_OFF_MASK                 (0b1 << 24)                             // B:24         For test purposes only; must be set to ‘0’
#define REG6_DSM_ORDER_OFFSET               22                                      // B:[23:22]    Set the order of delta-sigma modulator
#define REG6_DSM_ORDER_3RD_MASK             (0b00 << REG6_DSM_ORDER_OFFSET)         // B:[23:22]    3rd order DSM (recommended)
#define REG6_DSM_ORDER_2ND_MASK             (0b01 << REG6_DSM_ORDER_OFFSET)         // B:[23:22]    2nd order DSM
#define REG6_DSM_ORDER_1ST_MASK             (0b10 << REG6_DSM_ORDER_OFFSET)         // B:[23:22]    1st order DSM
#define REG6_DSM_ORDER_4TH_MASK             (0b11 << REG6_DSM_ORDER_OFFSET)         // B:[23:22]    4th order DSM
#define REG6_RESERVED_MASK                  (0b1 << 21)                             // B:21         Reserved, leave at 0
#define REG6_EN_AUTOCAL_MASK                (0b1 << 20)                             // B:20         Enable the VCO calibration auto-restart feature (0 = off, 1 = on)
#define REG6_VCO_SEL_OFFSET                 18                                      // B:[19:18]    VCO selection bits. For test purposes only.
#define REG6_VCO_SEL_MASK                   (0b11 << REG6_VCO_SEL_OFFSET)           // B:[19:18]    VCO selection bits - 0 to 3. For test purposes only. (disabled by auto-calibration)
#define REG6_VCO_WORD_OFFSET                13                                      // B:[17:13]    Select specific VCO sub-band (range:0 to 31). For test purposes only.
#define REG6_VCO_WORD_MASK                  (0x1F << REG6_VCO_WORD_OFFSET)          // B:[17:13]    Select specific VCO sub-band (range:0 to 31). For test purposes only. (disabled by auto-calibration)
#define REG6_CAL_TEMP_COMP_MASK             (0b1 << 12)                             // B:12         Enable temp compensation of VCO calibration procedure - when PLL Lock condition is required on extreme thermal cycles (0 = disabled - switching-frequency applications, 1 = on - fixed-frequency applications)
#define REG6_PRCHG_DEL_OFFSET               10                                      // B:[11:10]    Set the number of calibration slots for pre-charge of VCTRL node at the voltage reference value used during VCO calibration procedure
#define REG6_PRCHG_DEL_1SLT_MASK            (0b00 << REG6_PRCHG_DEL_OFFSET)         // B:[11:10]    1 slot (default)
#define REG6_PRCHG_DEL_2SLT_MASK            (0b01 << REG6_PRCHG_DEL_OFFSET)         // B:[11:10]    2 slots
#define REG6_PRCHG_DEL_3SLT_MASK            (0b10 << REG6_PRCHG_DEL_OFFSET)         // B:[11:10]    3 slots
#define REG6_PRCHG_DEL_4SLT_MASK            (0b11 << REG6_PRCHG_DEL_OFFSET)         // B:[11:10]    4 slots
#define REG6_CAL_ACC_EN_MASK                (0b1 << 9)                              // B:9          Increase calibrator accuracy by removing residual error taking 2 additional calibration slots (default = 0)
#define REG6_CAL_DIV_OFFSET                 0                                       // B:[8:0]      Calibrator clock divider ratio
#define REG6_CAL_DIV_MASK                   (0x1FF << REG6_CAL_DIV_OFFSET)          // B:[8:0]      Set Calibrator clock divider ratio (1 to 511); 0 = max (511)

#define REG6_CAL_DIV_MIN                    1                                       // Minimum CAL_DIV value
#define REG6_CAL_DIV_MAX                    511                                     // Maximum CAL_DIV value
#define REG6_CAL_DIV_RANGE                  REG6_CAL_DIV_MIN, REG6_CAL_DIV_MAX      // To be used as the second pair of parameters in in_range()

// Register 7 options (Fast Lock control, LD_SDO settings)
#define REG7_RESERVED_OFFSET                26                                      // B:26         Reserved, leave at 0
#define REG7_RESERVED_MASK                  (0b1 << REG7_RESERVED_OFFSET)           // B:26         Reserved, leave at 0
#define REG7_LD_SDO_tristate_MASK           (0b1 << 25)                             // B:25         LD_SDO tristate enable (0 = tristate disabled, 1 = tristate enabled)
#define REG7_LD_SDO_MODE_MASK               (0b1 << 24)                             // B:24         LD_SDO mode control (0 = Open Drain mode (Level Range: 1.8 V to 3.6 V), 1 = 2.5V CMOS output mode)
#define REG7_SPI_DATA_OUT_DISABLE_MASK      (0b1 << 23)                             // B:23         Disable auto-switch of LD_SDO pin during SPI read mode (0 = auto-switch enabled, 1 = fixed to Lock detector indication (SPI read operation not possible))
#define REG7_LD_SDO_SEL_OFFSET              21                                      // B:[22:21]    LD_SDO mux output selection bits
#define REG7_LD_SDO_SEL_LOCK_DETECT_MASK    (0b00 << REG7_LD_SDO_SEL_OFFSET)        // B:[22:21]    Lock detector (default)
#define REG7_LD_SDO_SEL_VCO_MASK            (0b01 << REG7_LD_SDO_SEL_OFFSET)        // B:[22:21]    VCO Divider output (for test purposes only)
#define REG7_LD_SDO_SEL_VCO_DIV_MASK        (0b10 << REG7_LD_SDO_SEL_OFFSET)        // B:[22:21]    Calibrator VCO Divider output (for test purposes only)
#define REG7_LD_SDO_SEL_FAST_LOCK_MASK      (0b11 << REG7_LD_SDO_SEL_OFFSET)        // B:[22:21]    Fast Lock indicator (for test purposes only)
#define REG7_REGDIG_OCP_DIS_MASK            (0b1 << 20)                             // B:20         Must be 0! Disable overcurrent protection of digital LDO (0 = enabled, 1 = disabled)
#define REG7_CYCLE_SLIP_EN_MASK             (0b1 << 19)                             // B:19         Cycle slip enable (0 = disabled, 1 = enabled)
#define REG7_FSTLCK_EN_MASK                 (0b1 << 18)                             // B:18         Fast lock mode enable using pin #6 (PD_RF2/FL_SW)
#define REG7_CP_SEL_FL_OFFSET               13                                      // B:[17:13]    Charge Pump current bit offset
#define REG7_CP_SEL_FL_MASK                 (0x1F << REG_CP_SEL_FL_OFFSET)          // B:[17:13]    Charge Pump current during fast lock time slot - 0 to 31
#define REG7_FSTLCK_CNT_OFFSET              0                                       // B:[12:0]     Fast lock counter value; set duration of fast-lock time slot as number of FPFD cycles
#define REG7_FSTLCK_CNT_MASK                0x1FFF                                  // B:[12:0]     Fast lock time slot counter - 2 to 8191

#define REG7_FSTLCK_CNT_MIN                 2                                          // Minimum FSTLCK_CNT value
#define REG7_FSTLCK_CNT_MAX                 8191                                       // Maximum FSTLCK_CNT value
#define REG7_FSTLCK_CNT_RANGE               REG7_FSTLCK_CNT_MIN, REG7_FSTLCK_CNT_MAX   // To be used as the second pair of parameters in in_range()

// Register 8 options (LDO Voltage Regulator settings)
#define REG8_RESERVED_OFFSET                3                                       // B:[25:20,15,11,7,3]  Reserved, leave at 0
#define REG8_RESERVED_MASK                  (0x3F0911 << REG8_RESERVED_OFFSET)      // B:[25:20,15,11,7,3]  Reserved, leave at 0
#define REG8_PD_RF2_DISABLE_MASK            (0b1 << 26)                             // B:26         Disable the hardware power down function of the pin PD_RF2 (pin #6) thus allowing the pin PD_RF1 (pin #5) to control the power down status of both RF output stages
#define REG8_REG_OCP_DIS_MASK               (0b1 << 19)                             // B:19         Must be 0! Disable overcurrent protection of all LDOs except DIG (0 = enabled, 1 = disabled)
#define REG8_REG_DIG_PD_MASK                (0b1 << 18)                             // B:18         Must be 0! Power down the digital LDO (0 = on, 1 = off)
#define REG8_REG_DIG_VOUT_OFFSET            16                                      // B:[17:16]    Digital regulator output voltage set bit offset
#define REG8_REG_DIG_VOUT_2_6V_MASK         (0b00 << REG8_REG_DIG_VOUT_OFFSET)      // B:[17:16]    2.6 V (default)
//#define REG8_REG_DIG_VOUT_2_3V_MASK       (0b01 << REG8_REG_DIG_VOUT_OFFSET)      // B:[17:16]    2.3 V (test purposes only)
//#define REG8_REG_DIG_VOUT_2_4V_MASK       (0b10 << REG8_REG_DIG_VOUT_OFFSET)      // B:[17:16]    2.4 V (test purposes only)
//#define REG8_REG_DIG_VOUT_2_5V_MASK       (0b11 << REG8_REG_DIG_VOUT_OFFSET)      // B:[17:16]    2.5 V (test purposes only)
#define REG8_REG_REF_PD_MASK                (0b1 << 14)                             // B:14         Must be 0! Power down the REF_CLK LDO (0 = on, 1 = off)
#define REG8_REG_REF_VOUT_OFFSET            12                                      // B:[13:12]    Reference clock regulator output voltage set bit offset
#define REG8_REG_REF_VOUT_2_6V_MASK         (0b00 << REG8_REG_REF_VOUT_OFFSET)      // B:[13:12]    2.6 V (default)
//#define REG8_REG_REF_VOUT_2_5V_MASK       (0b01 << REG8_REG_REF_VOUT_OFFSET)      // B:[13:12]    2.5 V (test purposes only)
//#define REG8_REG_REF_VOUT_2_7V_MASK       (0b10 << REG8_REG_REF_VOUT_OFFSET)      // B:[13:12]    2.7 V (test purposes only)
//#define REG8_REG_REF_VOUT_2_8V_MASK       (0b11 << REG8_REG_REF_VOUT_OFFSET)      // B:[13:12]    2.8 V (test purposes only)
#define REG8_REG_RF_PD_MASK                 (0b1 << 10)                             // B:10         Must be 0! Power down the RF output LDO (0 = on, 1 = off)
#define REG8_REG_RF_VOUT_OFFSET             8                                       // B:[9:8]      RF output regulator output voltage set bit offset
#define REG8_REG_RF_VOUT_2_6V_MASK          (0b00 << REG8_REG_RF_VOUT_OFFSET)       // B:[9:8]      2.6 V (default)
//#define REG8_REG_RF_VOUT_2_5V_MASK        (0b01 << REG8_REG_RF_VOUT_OFFSET)       // B:[9:8]      2.5 V (test purposes only)
//#define REG8_REG_RF_VOUT_2_7V_MASK        (0b10 << REG8_REG_RF_VOUT_OFFSET)       // B:[9:8]      2.7 V (test purposes only)
//#define REG8_REG_RF_VOUT_2_8V_MASK        (0b11 << REG8_REG_RF_VOUT_OFFSET)       // B:[9:8]      2.8 V (test purposes only)
#define REG8_REG_VCO_PD_MASK                (0b1 << 6)                              // B:6          Must be 0! Power down the VCO bias-and-control regulator (0 = on, 1 = off)
#define REG8_REG_VCO_VOUT_OFFSET            4                                       // B:[5:4]      VCO bias-and-control regulator output voltage set bit offset
#define REG8_REG_VCO_VOUT_2_6V_MASK         (0b00 << REG8_REG_VCO_VOUT_OFFSET)      // B:[5:4]      2.6 V (default)
//#define REG8_REG_VCO_VOUT_2_5V_MASK       (0b01 << REG8_REG_VCO_VOUT_OFFSET)      // B:[5:4]      2.5 V (test purposes only)
//#define REG8_REG_VCO_VOUT_2_7V_MASK       (0b10 << REG8_REG_VCO_VOUT_OFFSET)      // B:[5:4]      2.7 V (test purposes only)
//#define REG8_REG_VCO_VOUT_2_8V_MASK       (0b11 << REG8_REG_VCO_VOUT_OFFSET)      // B:[5:4]      2.8 V (test purposes only)
#define REG8_REG_VCO_4V5_PD_MASK            (0b1 << 2)                              // B:2          Must be 0! Power down the high-voltage regulator to be used to supply VCO core, RF output final stage and Charge Pump (0 = on, 1 = off)
#define REG8_REG_VCO_4V5_VOUT_OFFSET        0                                       // B:[1:0]      High-voltage regulator output voltage set bit offset
//#define REG8_REG_VCO_4V5_VOUT_5_0V_MASK   (0b00 << REG8_REG_VCO_4V5_VOUT_OFFSET)  // B:[1:0]      5.0 V (Requires 5.4 V unregulated voltage line on pin #36, for test purposes only)
//#define REG8_REG_VCO_4V5_VOUT_2_6V_MASK   (0b01 << REG8_REG_VCO_4V5_VOUT_OFFSET)  // B:[1:0]      2.6 V (3.0 - 5.4 V unregulated voltage line range allowed on pin #36, for test purposes only)
#define REG8_REG_VCO_4V5_VOUT_3_3V_MASK     (0b10 << REG8_REG_VCO_4V5_VOUT_OFFSET)  // B:[1:0]      3.3 V (3.6 - 5.4 V unregulated voltage line range allowed on pin #36)
#define REG8_REG_VCO_4V5_VOUT_4_5V_MASK     (0b11 << REG8_REG_VCO_4V5_VOUT_OFFSET)  // B:[1:0]      4.5 V (5.0 - 5.4 V unregulated voltage line range allowed on pin #36)

// Register 9 options (Reserved (Test and Initialization bit))
#define REG9_RESERVED_MASK 0xFFFFFFFF // The entire register must be set to 0

// Register 10 read only (VCO, Lock det. Status, LDO status)
#define REG10_RESERVED_OFFSET               18                                      // B:[26:18]    Reserved, leave at 0
#define REG10_RESERVED_MASK                 (0x1FF << REG10_RESERVED_OFFSET)        // B:[26:18]    Reserved, leave at 0
#define REG10_REG_DIG_STARTUP_MASK          (0b1 << 17)                             // B:17         DIGITAL regulator ramp-up indicator (1 means correct start-up)
#define REG10_REG_REF_STARTUP_MASK          (0b1 << 16)                             // B:16         REF_CLK regulator ramp-up indicator (1 means correct start-up)
#define REG10_REG_RF_STARTUP_MASK           (0b1 << 15)                             // B:15         RF Output section regulator ramp-up indicator  (1 means correct start-up)
#define REG10_REG_VCO_STARTUP_MASK          (0b1 << 14)                             // B:14         VCO bias-and-control regulator ramp-up indicator (1 means correct start-up)
#define REG10_REG_VCO_4V5_STARTUP_MASK      (0b1 << 13)                             // B:13         High-voltage regulator ramp-up indicator (1 means correct start-up)
#define REG10_REG_DIG_OCP_MASK              (0b1 << 12)                             // B:12         DIGITAL regulator overcurrent protection indicator (1 means overcurrent)
#define REG10_REG_REF_OCP_MASK              (0b1 << 11)                             // B:11         REF_CLK regulator overcurrent protection indicator (1 means overcurrent)
#define REG10_REG_RF_OCP_MASK               (0b1 << 10)                             // B:10         RF Output section regulator overcurrent protection indicator (1 means overcurrent)
#define REG10_REG_VCO_OCP_MASK              (0b1 << 9)                              // B:9          VCO bias-and-control regulator overcurrent protection indicator (1 means overcurrent)
#define REG10_REG_VCO_4V5_OCP_MASK          (0b1 << 8)                              // B:8          High-voltage regulator overcurrent protection indicator (1 means overcurrent)
#define REG10_LOCK_DET_MASK                 (0b1 << 7)                              // B:7          Lock detector status (1 means PLL locked)
#define REG10_VCO_SEL_OFFSET                5                                       // B:[6:5]      VCO selection bits
#define REG10_VCO_SEL_MASK                  (0b11 << REG10_VCO_SEL_OFFSET)          // B:[6:5]      VCO selection bits 0 to 3
#define REG10_WORD_OFFSET                   0                                       // B:[4:0]      VCO word bits
#define REG10_WORD_MASK                     0x1F                                    // B:[4:0]      Specific VCO sub-band selected by calibration algorithm - 0 to 31

// Register 11 read only (Device ID)
#define REG11_Device_ID_MASK                0xFFFFFFFF                              // B:[26:0]     Device ID


// Some define jugglery

// Leave this part alone
#ifdef CORE_VCC_4V5
    #define CORE_VCC_HIGH_MASK           0xFFFFFFFF  // Mask for 4.5 V settings
    #define CORE_VCC_LOW_MASK            0x00000000  // Mask for 3.3 V settings
    #define FLAG_3V3                     false       // Flag for 3.3 V settings
#endif
#ifdef CORE_VCC_3V3
    #define CORE_VCC_HIGH_MASK           0x00000000  // Mask for 4.5 V settings
    #define CORE_VCC_LOW_MASK            0xFFFFFFFF  // Mask for 3.3 V settings
    #define FLAG_3V3                     true        // Flag for 3.3 V settings
#endif

#ifdef USE_DOUBLE_BUFFERING
    #define DBR_MASK                    0xFFFFFFFF  // Mask for double buffering enabled
#else
    #define DBR_MASK                    0x00000000  // Mask for double buffering disabled
#endif

// End of leave this part alone

// This needs to be visible from C code
#ifdef __cplusplus
extern "C" {
#endif

// SCPI commands
extern scpi_choice_def_t scpi_choice_frequency_units[];

// Function prototypes
void send_STUW81300(uint32_t addr, uint32_t data);
uint32_t read_STUW81300(uint32_t addr);

uint64_t update_STUW81300(uint64_t frequency, bool mute);
uint64_t set_frequency_STUW81300(uint64_t frequency);

void mute_STUW81300(bool mute);
bool is_muted_STUW81300(void);

bool check_lock_STUW81300(void);

void init_STUW81300(bool busy_wait);

#ifdef __cplusplus
}
#endif

#endif /* STUW81300_H_INCLUDED */
