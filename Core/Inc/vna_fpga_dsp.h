#ifndef VNA_FPGA_DSP_H
#define VNA_FPGA_DSP_H

#include "main.h"
#include "types.h"
#include "vna_meas_data.h"
#include "vna_tx_utils.h"

// SPI settings (use SPI (hspi4 on H5))
#define DSP_SPI                     hspi4
//#define DSP_RST_PORT - whoops, DSP board revision 0.1 doesn't have a reset pin
//#define DSP_RST_PIN - whoops, DSP board revision 0.1 doesn't have a reset pin

#define DSP_CS_FPGA_PORT            FPGA_CS_GPIO_Port
#define DSP_CS_FPGA_PIN             FPGA_CS_Pin
#define DSP_CS_FLASH_PORT           SPI4_NSS_GPIO_Port
#define DSP_CS_FLASH_PIN            SPI4_NSS_Pin

#define DSP_CONVERSION_DONE_PORT    CONVERSION_DONE_GPIO_Port
#define DSP_CONVERSION_DONE_PIN     CONVERSION_DONE_Pin

#define DSP_SPI_DATA_SIZE           1 // in chunks defined in the HAL SPI library
#define DSP_SPI_DELAY               50 // in us, should be enough
#define DSP_SPI_TIMEOUT             10 // in ms, should be enough

// FPGA specific status pins
#define DSP_CDONE_PORT              FPGA_CRESET_GPIO_Port
#define DSP_CDONE_PIN               FPGA_CRESET_Pin
#define DSP_CRESET_PORT             FPGA_CDONE_GPIO_Port
#define DSP_CRESET_PIN              FPGA_CDONE_Pin

// Register component shifts and masks
// TODO - this is a placeholder for the STUW81300, replace with the actual DSP register definitions
#define DSP_RW_MASK           0x80000000                                      // R/W bit (1 = Read, 0 = Write)
#define DSP_ADDR_MASK         0x0F                                            // Address bits before shifting (bits 3 to 0)
#define DSP_ADDR_SHIFT        27                                              // Shift for address bits
#define DSP_ADDR_MASK_SH      (STUW81300_ADDR_MASK << STUW81300_ADDR_SHIFT)   // Address bits (bits 30 to 27)
#define DSP_DATA_MASK         0x07FFFFFF                                      // Data bits (bits 26 to 0)

// DSP limits
#define DSP_MAX_COEFFICIENTS   16

#ifdef __cplusplus
extern "C" {
#endif

void init_FPGA_DSP();

// Function prototypes
void send_DSP(uint32_t addr, uint32_t data);
uint32_t read_DSP(uint32_t addr);

int32_t dsp_set_rbw_filter_coefficient(int32_t coefficient, uint32_t coef_index);
int32_t dsp_set_rbw_filter_coefficients(const int32_t *coefficients, uint32_t num_coefficients);

int32_t dsp_get_rbw_filter_coefficient(int32_t *coefficient, uint32_t coef_index);

int32_t dsp_start_measurement(uint64_t freq);
int32_t dsp_read_point(meas_point_t *point);

#ifdef __cplusplus
}
#endif

#endif //VNA_FPGA_DSP_H
