//
// Created by matej on 30/08/2024.
//

#ifndef VNA_CMD_ENGINE_H
#define VNA_CMD_ENGINE_H

#include "main.h"

#include "tx_api.h"
#include "tx_port.h"

#include "scpi/scpi.h"

#include "vna_signaling.h"
#include "vna_tx_utils.h"

// Defaults
#define ECHO_DEFAULT                false

// Message queues
#define QUEUE_STACK_SIZE            128

// SCPI Defines
#define SCPI_INPUT_BUFFER_LENGTH    256
#define SCPI_ERROR_QUEUE_SIZE       17

#define SCPI_IDN1   "Matej Pevec (LSO)"
#define SCPI_IDN2   "8-16 GHz Open Source VNA"
#define SCPI_IDN3   "DEV0001"
#define SCPI_IDN4   "0.1.0"

// This needs to be visible from C code
#ifdef __cplusplus
extern "C" {
#endif

// C visible global variables //
// Message queues
extern uint8_t out_queue_stack[QUEUE_STACK_SIZE];
extern TX_QUEUE out_queue;
//extern uint8_t in_queue_stack[QUEUE_STACK_SIZE];
//extern TX_QUEUE in_queue;

// SCPI core variables
extern const scpi_command_t scpi_commands[];
extern scpi_interface_t scpi_interface;
extern char scpi_input_buffer[];
extern scpi_error_t scpi_error_queue_data[];
extern scpi_t scpi_context;

// Command engine state variables
extern bool echo_enabled;

// C visible functions //
// ThreadX related
void vna_init_cmd_engine();
void vna_process_command(const UCHAR* buffer, ULONG length);

// SCPI core functions
size_t SCPI_Write(scpi_t* context, const char* data, size_t len);
int SCPI_Error(scpi_t * context, int_fast16_t err);
scpi_result_t SCPI_Control(scpi_t * context, scpi_ctrl_name_t ctrl, scpi_reg_val_t val);
scpi_result_t SCPI_Reset(scpi_t * context);
scpi_result_t SCPI_Flush(scpi_t * context);

// SCPI commands


#ifdef __cplusplus
}
#endif

#endif //VNA_CMD_ENGINE_H
