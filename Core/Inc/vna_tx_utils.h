//
// Created by matej on 01/09/2024.
//

#ifndef VNA_TX_UTILS_H
#define VNA_TX_UTILS_H

#include "main.h"
#include "tx_api.h"

#include "vna_meas_data.h"

// Config (microsecond timer)
#define US_TIMER htim1

// This needs to be visible from C code
#ifdef __cplusplus
extern "C" {
#endif

// VNA message types
typedef enum vna_msg_type {
    VNA_MSG_NONE,
    VNA_MSG_TEXT,
    VNA_MSG_MEAS_DATA,
} vna_msg_type;

// Function prototypes
void init_utils();

void delay_us(int us);

bool in_range64(uint64_t value, uint64_t min, uint64_t max);
bool in_range(uint32_t value, uint32_t min, uint32_t max);
uint32_t limit(uint32_t value, uint32_t min, uint32_t max);

size_t appendCharArray(char* destination, size_t destinationSize, const char* source);
size_t intToCharArray(int value, char* buffer, size_t bufferSize, bool terminate);

// VNA message queue functions
vna_msg_type get_next_msg_type(TX_QUEUE* char_queue);
size_t get_next_msg_len(TX_QUEUE* char_queue);

size_t send_text_to_queue(TX_QUEUE* char_queue, const char* data, size_t len);
size_t receive_text_from_queue(TX_QUEUE* char_queue, char* buffer, size_t len);

size_t send_meas_data_to_queue(TX_QUEUE* char_queue);

#ifdef __cplusplus
}
#endif
#endif //VNA_TX_UTILS_H
