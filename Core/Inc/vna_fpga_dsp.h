#ifndef VNA_FPGA_DSP_H
#define VNA_FPGA_DSP_H

#include "main.h"
#include "types.h"

#include "vna_meas_data.h"

#include "vna_tx_utils.h"
#include "vna_dma_utils.h"

// General settings
#define DSP_MEAS_POINT_CNT  4096 // Number of measurement points

// SPI settings (use SPI (hspi4 on H5))
#define DSP_SPI                     hspi4
#define DSP_SPI_INST                SPI4
//#define DSP_RST_PORT - whoops, DSP board revision 0.1 doesn't have a reset pin
//#define DSP_RST_PIN - whoops, DSP board revision 0.1 doesn't have a reset pin

#define DSP_CS_FPGA_PORT            FPGA_CS_GPIO_Port
#define DSP_CS_FPGA_PIN             FPGA_CS_Pin
#define DSP_CS_FLASH_PORT           SPI4_NSS_GPIO_Port
#define DSP_CS_FLASH_PIN            SPI4_NSS_Pin
#define DSP_SCLK_PORT               SPI4_SCK_GPIO_Port
#define DSP_SCLK_PIN                SPI4_SCK_Pin
#define DSP_MISO_PORT               SPI4_MISO_GPIO_Port
#define DSP_MISO_PIN                SPI4_MISO_Pin
#define DSP_MOSI_PORT               SPI4_MOSI_GPIO_Port
#define DSP_MOSI_PIN                SPI4_MOSI_Pin

// Doubles as a "busy" pin when the DSP is converting or reading data
#define DSP_CONVERSION_DONE_PORT    CONVERSION_DONE_GPIO_Port
#define DSP_CONVERSION_DONE_PIN     CONVERSION_DONE_Pin

#define DSP_SPI_DATA_SIZE           4 // in chunks defined in the HAL SPI library (8 bits here)
#define DSP_SPI_DELAY               0 // in us
#define DSP_SPI_TIMEOUT             2 // in ms

// FPGA specific status pins
#define DSP_CDONE_PORT              FPGA_CRESET_GPIO_Port
#define DSP_CDONE_PIN               FPGA_CRESET_Pin
#define DSP_CRESET_PORT             FPGA_CDONE_GPIO_Port
#define DSP_CRESET_PIN              FPGA_CDONE_Pin

// Register component shifts and masks
#define DSP_RW_MASK           0x80000000                                      // R/W bit (1 = Read, 0 = Write)
#define DSP_ADDR_MASK         0x7F                                            // Address bits before shifting (bits 6 to 0)
#define DSP_ADDR_SHIFT        24                                              // Shift for address bits
#define DSP_ADDR_MASK_SH      (STUW81300_ADDR_MASK << STUW81300_ADDR_SHIFT)   // Address bits (bits 30 to 24)
#define DSP_DATA_MASK         0x00FFFFFF                                      // Data bits (bits 23 to 0)

/*
# Registry Information
This document describes the FPGA configuration and data readout.
## Registry map
| Name					| Address	| Direction	| Map											|
| :-------------------- | :-------: | :-------: | :--------------------------------------------	|
| General settings		| 0000000	| W		| [23:2]: 0, 1: FIR flush[^1], 0: RST
| General status		| 0000001	| R		| [23:1]: 0, 0: PLL lock
| Conversion control	| 0000010	| R/W	| [23:14]: 0, 13: Start conversion, [12:0]: Point count
| Conversion status		| 0000011	| R		| [23:14]: 0, R13: Conversion done, [12:0]: Converted point count
| Readout 				| 0000100	| R/W	| [R23:0]: Next data \| [W23:1]: 0, W0: Restart readout
| Readout status		| 0000101	| R/W	| [23:14]: 0, 13: Readout done, [12:0] Current index
| FIR Shift[^1]			| 0100000	| R/W	| [23:5]: 0, [4:0]: FIR shift (divider 2^n)
| Previous transaction	| 0111110	| R		| [23:0]: Last data received by the FPGA
| Device ID				| 0111111	| R		| [23:0]: device ID
| FIR Coefs[^1]			|100xxxx[^2]| R/W	| [23:0]: FIR coefficients

[^1]: This option is currently disabled due to the limited implementation
[^2]: The `xxxx` part is the coefficient index for this item
*/

// DSP registers
#define DSP_REG_GEN_SETTINGS        0b0000000       // General settings
#define DSP_REG_GEN_STATUS          0b0000001       // General status
#define DSP_REG_CONV_CONTROL        0b0000010       // Conversion control
#define DSP_REG_CONV_STATUS         0b0000011       // Conversion status
#define DSP_REG_READOUT             0b0000100       // Readout
#define DSP_REG_READOUT_STATUS      0b0000101       // Readout status
#define DSP_REG_FIR_SHIFT           0b0100000       // FIR Shift (currently not implemented)
#define DSP_REG_PREV_TRANSACTION    0b0111110       // Previous transaction
#define DSP_REG_DEVICE_ID           0b0111111       // Device ID
#define DSP_REG_FIR_COEF            0b1000000       // FIR Coefficients (currently not implemented)

// DSP general settings (FIR flush (not implemented), reset)
#define DSP_GEN_SET_RESERVED_OFFSET     2               // Reserved bits offset
#define DSP_GEN_SET_RESERVED_MASK       (0x3FFFFF << DSP_GEN_SET_RESERVED_OFFSET) // Reserved bits
#define DSP_GEN_SET_FIR_FLUSH           (1 << 1)      // 1: FIR_FLUSH
#define DSP_GEN_SET_RST                 (1 << 0)      // 0: RST

// DSP general status (PLL lock - internal clock locked to the external reference)
#define DSP_GEN_STAT_RESERVED_OFFSET    1              // Reserved bits offset
#define DSP_GEN_STAT_RESERVED_MASK      (0x7FFFFF << DSP_GEN_STAT_RESERVED_OFFSET) // Reserved bits
#define DSP_GEN_STAT_PLL_LOCK           (1 << 0)     // 0: PLL lock

// DSP conversion control (start conversion)
#define DSP_CONV_CTRL_RESERVED_OFFSET   14            // Reserved bits offset
#define DSP_CONV_CTRL_RESERVED_MASK     (0x3FFFFF << DSP_CONV_CTRL_RESERVED_OFFSET) // Reserved bits
#define DSP_CONV_CTRL_START_CONV        (1 << 13)   // 13: Start conversion
#define DSP_CONV_CTRL_POINT_CNT_MASK    0x1FFF        // [12:0]: Point count mask (leave at 0 for previous value)

// DSP conversion status (conversion done, point count)
#define DSP_CONV_STAT_RESERVED_OFFSET   14            // Reserved bits offset
#define DSP_CONV_STAT_RESERVED_MASK     (0x7FF << DSP_CONV_STAT_RESERVED_OFFSET) // Reserved bits
#define DSP_CONV_STAT_CONV_DONE         (1 << 13)   // 13: Conversion done
#define DSP_CONV_STAT_POINT_CNT_MASK    0x1FFF        // [12:0]: Point count mask (leave at 0 for maximum value)

// DSP readout (next data, restart readout)
#define DSP_READOUT_RESTART_READOUT     (1 << 0)    // 0: Restart readout
#define DSP_READOUT_NEXT_DATA_MASK      0xFFFFFF      // [23:0]: Next data mask
#define DSP_READOUT_MEAS_MASK           0xFFF         // [11:0]: Measurement mask
#define DSP_READOUT_MEAS_A_OFFSET       12            // Measurement A input
#define DSP_READOUT_MEAS_B_OFFSET       0             // Measurement B input

// DSP readout status (readout done, current index)
#define DSP_READOUT_STAT_RESERVED_OFFSET 14         // Reserved bits offset
#define DSP_READOUT_STAT_RESERVED_MASK  (0x7FF << DSP_READOUT_STAT_RESERVED_OFFSET) // Reserved bits
#define DSP_READOUT_STAT_READOUT_DONE   (0b1 << 13) // 13: Readout done
#define DSP_READOUT_STAT_CURR_IDX_MASK  0x1FFF       // [12:0]: Current index mask

// DSP FIR shift (divider 2^n)
#define DSP_FIR_SHIFT_RESERVED_OFFSET   5              // Reserved bits offset
#define DSP_FIR_SHIFT_RESERVED_MASK     (0x7FFFFF << DSP_FIR_SHIFT_RESERVED_OFFSET) // Reserved bits
#define DSP_FIR_SHIFT_FIR_SHIFT_MASK    0x1F           // [4:0]: FIR shift mask (divider 2^n)

// DSP device ID
#define DSP_DEVICE_ID_MASK              0xFFFFFF       // [23:0]: Device ID mask

// DSP FIR coefficients
#define DSP_FIR_COEF_INDEX_MASK         0xFFFFFF       // [23:0]: FIR coefficient index mask

// DSP limits
#define DSP_MAX_COEFFICIENTS   16

// Raw sample storage
extern adc_point_t raw_samples[DSP_MEAS_POINT_CNT];
extern int32_t raw_sample_count;

#ifdef __cplusplus
extern "C" {
#endif

void init_FPGA_DSP();

uint32_t read_previous_data_sent();
void inf_circular_reg_read();

// Function prototypes
void send_DSP(uint32_t addr, uint32_t data);
uint32_t read_DSP(uint32_t addr);

int32_t dsp_set_rbw_filter_coefficient(int32_t coefficient, uint32_t coef_index);
int32_t dsp_set_rbw_filter_coefficients(const int32_t *coefficients, uint32_t num_coefficients);

int32_t dsp_get_rbw_filter_coefficient(int32_t *coefficient, uint32_t coef_index);

int32_t dsp_start_measurement(uint64_t freq);
int32_t dsp_read_point(meas_point_t *point);

int32_t dsp_start_sample_measurement();
void dsp_wait_for_measurement_done();
int32_t dsp_read_sample_point(adc_point_t *point);
int32_t dsp_read_all_points();

void dsp_spi_init(void);
void dsp_fpga_reinit(void);
void dsp_gpio_reinit();

bool dsp_is_busy();

#ifdef __cplusplus
}
#endif

#endif //VNA_FPGA_DSP_H
