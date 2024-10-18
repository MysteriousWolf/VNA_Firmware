//
// Created by matej on 11/10/2024.
//

#include "vna_dma_utils.h"

/**
 * Wait for the specified flag to be set
 * @param flag Flag to wait for
 */
void vna_wait_for_flag(const ULONG flag) {
    ULONG actual_flags;
    tx_event_flags_get(&measurement_event_flags, flag, TX_AND_CLEAR, &actual_flags, TX_WAIT_FOREVER);
}

/**
 * Release the specified flag
 * @param flag Flag to release
 */
void vna_release_flag(const ULONG flag) {
    tx_event_flags_set(&measurement_event_flags, flag, TX_OR);
}

/**
 * Universal DMA transfer complete handler
 * @param hspi SPI handle
 */
static void dma_transfer_complete(const SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == DSP_SPI_INST) {
        vna_release_flag(DSP_DMA_SPI_DONE);
    } else if (hspi->Instance == ADC_SPI_INST) {
        vna_release_flag(ADC_DMA_SPI_DONE);
    } else if (hspi->Instance == PLL_SPI_INST) {
        vna_release_flag(PLL_DMA_SPI_DONE);
    }
}

/**
 * SPI DMA TxRx transfer complete callback
 * @param hspi SPI handle
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    dma_transfer_complete(hspi);
}

/**
 * SPI DMA Rx transfer complete callback
 * @param hspi SPI handle
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    dma_transfer_complete(hspi);
}

/**
 * SPI DMA Tx transfer complete callback
 * @param hspi SPI handle
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    dma_transfer_complete(hspi);
}