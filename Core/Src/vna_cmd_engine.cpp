//
// Created by matej on 30/08/2024.
//

#include "vna_cmd_engine.h"

#include "vna_stuw81300.h"

uint8_t out_queue_stack[QUEUE_STACK_SIZE];
TX_QUEUE out_queue;
//uint8_t in_queue_stack[QUEUE_STACK_SIZE];
//TX_QUEUE in_queue;

scpi_t scpi_context;

char scpi_input_buffer[SCPI_INPUT_BUFFER_LENGTH];
scpi_error_t scpi_error_queue_data[SCPI_ERROR_QUEUE_SIZE];

// Init global settings
bool echo_enabled = ECHO_DEFAULT;

void vna_init_cmd_engine() {
    // Initialize the message queues
    tx_queue_create(&out_queue, (char*)"SCPI Out Queue", 1, out_queue_stack, QUEUE_STACK_SIZE);
    //tx_queue_create(&in_queue, (char*)"SCPI In Queue", 1, in_queue_stack, QUEUE_STACK_SIZE);

    // Initialize the SCPI parser
    SCPI_Init(&scpi_context, scpi_commands, &scpi_interface, scpi_units_def,
              SCPI_IDN1, SCPI_IDN2, SCPI_IDN3, SCPI_IDN4,
              scpi_input_buffer, SCPI_INPUT_BUFFER_LENGTH,
              scpi_error_queue_data, SCPI_ERROR_QUEUE_SIZE);
}

void vna_process_command(UCHAR* buffer, ULONG length) {
    // Echo the command if enabled
    if (echo_enabled)
        send_data_to_queue(&out_queue, (const char*)buffer, length);
    // Process the command
    SCPI_Input(&scpi_context, (const char*)buffer, static_cast<int>(length));
}

// SCPI core functions
size_t SCPI_Write(scpi_t* context, const char* data, size_t len) {
    (void)context; // Unused?

    // Send the data to the message queue in packed form
    return send_data_to_queue(&out_queue, data, len);
}

int SCPI_Error(scpi_t* context, int_fast16_t err) {
    (void)context; // Unused?

    char err_str[17] = "ERROR: ";
    char err_code[7];

    // Build the error message
    intToCharArray(err, err_code, 7, true);
    appendCharArray(err_str, 16, err_code);
    size_t len = appendCharArray(err_str, 16, "\r\n");

    // Send the error to the message queue in packed form
    return static_cast<int>(send_data_to_queue(&out_queue, err_str, len));
}

/**
 * Reimplement IEEE488.2 *TST?
 *
 * Result should be 0 if everything is ok
 * Result should be 1 if something goes wrong
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t VNA_CoreTstQ(scpi_t* context) {
    // TODO: Implement the self-test
    SCPI_ResultInt32(context, 0);
    return SCPI_RES_OK;
}

// Extended SCPI commands
static scpi_result_t VNA_CoreEcho(scpi_t* context) {
    bool echo_param;

    /* read first parameter if present */
    if (SCPI_ParamBool(context, &echo_param, false)) {
        echo_enabled = echo_param;
    }
    else {
        echo_enabled = !echo_enabled;
    }

    SCPI_ResultBool(context, echo_enabled);
    return SCPI_RES_OK;
}

static scpi_result_t VNA_DebugLED(scpi_t* context) {
    int32_t ld_id, ld_state;

    if (!SCPI_ParamInt32(context, &ld_id, true)) {
        return SCPI_RES_ERR;
    }

    if (!SCPI_ParamChoice(context, scpi_led_states, &ld_state, true)) {
        return SCPI_RES_ERR;
    }

    const auto state = static_cast<led_states>(ld_state);

    if (!set_led_state(ld_id, state))
        return SCPI_RES_ERR;

    return SCPI_RES_OK;
}

// VCO debug commands
static scpi_result_t VNA_Debug_VCO_FREQ(scpi_t* context) {
    uint64_t frequency;
    int32_t multiplier;

    // Parse the numeric part of the command
    if (!SCPI_ParamUInt64(context, &frequency, true)) {
        // Return an error code if parsing fails
        SCPI_ResultInt64(context, -1);
        return SCPI_RES_ERR;
    }

    // Check for unit parameters and convert
    if (!SCPI_ParamChoice(context, scpi_choice_frequency_units, &multiplier, false))
        multiplier = 1; // Default to Hz if no unit is provided

    // Convert the frequency to Hz
    frequency *= multiplier;

    // Ensure frequency is within the valid range for your instrument
    if (!in_range64(frequency, FREQ_RANGE)) {
        // Return an error code if out of range
        SCPI_ResultInt64(context, -2);
        return SCPI_RES_ERR;
    }

    // Set the frequency using your existing method
    const auto actual_frequency = static_cast<int64_t>(set_frequency_STUW81300(frequency));

    // Return the actual frequency set back to the user
    SCPI_ResultInt64(context, actual_frequency);

    if (actual_frequency == 0)
        return SCPI_RES_ERR;
    return SCPI_RES_OK;
}

static scpi_result_t VNA_Debug_VCO_MUTE(scpi_t* context) {
    bool mute_param;

    // Read the mute parameter
    if (SCPI_ParamBool(context, &mute_param, false)) {
        mute_STUW81300(mute_param);
    }
    else {
        // If none is provided, toggle the mute state
        mute_param = !is_muted_STUW81300();
        mute_STUW81300(mute_param);
    }

    // Check if it's actually muted and report the state back to the user
    SCPI_ResultBool(context, is_muted_STUW81300());
    return SCPI_RES_OK;
}

// VNA SCPI command list
// TODO: Add VNA-specific commands

scpi_interface_t scpi_interface = {
    .error = SCPI_Error,
    .write = SCPI_Write,
    .control = NULL,
    .flush = NULL,
    .reset = NULL,
};

const scpi_command_t scpi_commands[] = {
    /* IEEE Mandated Commands (SCPI std V1999.0 4.1.1) */
    {.pattern = "*CLS", .callback = SCPI_CoreCls,},
    {.pattern = "*ESE", .callback = SCPI_CoreEse,},
    {.pattern = "*ESE?", .callback = SCPI_CoreEseQ,},
    {.pattern = "*ESR?", .callback = SCPI_CoreEsrQ,},
    {.pattern = "*IDN?", .callback = SCPI_CoreIdnQ,},
    {.pattern = "*OPC", .callback = SCPI_CoreOpc,},
    {.pattern = "*OPC?", .callback = SCPI_CoreOpcQ,},
    {.pattern = "*RST", .callback = SCPI_CoreRst,},
    {.pattern = "*SRE", .callback = SCPI_CoreSre,},
    {.pattern = "*SRE?", .callback = SCPI_CoreSreQ,},
    {.pattern = "*STB?", .callback = SCPI_CoreStbQ,},
    {.pattern = "*TST?", .callback = VNA_CoreTstQ,},
    {.pattern = "*WAI", .callback = SCPI_CoreWai,},

    /* Required SCPI commands (SCPI std V1999.0 4.2.1) */
    {.pattern = "SYSTem:ERRor[:NEXT]?", .callback = SCPI_SystemErrorNextQ,},
    {.pattern = "SYSTem:ERRor:COUNt?", .callback = SCPI_SystemErrorCountQ,},
    {.pattern = "SYSTem:VERSion?", .callback = SCPI_SystemVersionQ,},

    {.pattern = "STATus:QUEStionable[:EVENt]?", .callback = SCPI_StatusQuestionableEventQ,},
    /* {.pattern = "STATus:QUEStionable:CONDition?", .callback = scpi_stub_callback,}, */
    {.pattern = "STATus:QUEStionable:ENABle", .callback = SCPI_StatusQuestionableEnable,},
    {.pattern = "STATus:QUEStionable:ENABle?", .callback = SCPI_StatusQuestionableEnableQ,},

    {.pattern = "STATus:PRESet", .callback = SCPI_StatusPreset,},

    /* Utility/System test commands */
    {.pattern = "*ECHo", .callback = VNA_CoreEcho,}, // Enable or disable echo (wonky but useful)
    {.pattern = "DEBug:LED", .callback = VNA_DebugLED,}, // Debug LED control
    {.pattern = "DEBug:VCO:FREQuency", .callback = VNA_Debug_VCO_FREQ,}, // Debug VCO frequency control
    {.pattern = "DEBug:VCO:MUTe", .callback = VNA_Debug_VCO_MUTE,}, // Debug VCO mute control

    /* VNA */
    // Calibration
    {.pattern = "SENSe:CORRection:ENable", .callback = NULL,}, // Enable correction data
    {.pattern = "SENSe:CORRection:ENable?", .callback = NULL,}, // Check if meas correction is enabled


    {.pattern = "SENSe:CORRection:STATus?", .callback = NULL,}, // Check if the correction data is valid
    {.pattern = "SENSe:CORRection:VALid?", .callback = NULL,}, // Checks if the current measurement data matches the current correction data

    {.pattern = "SENSe:CORRection:SELECT", .callback = NULL,}, // Select the active calibration ID
    {.pattern = "SENSe:CORRection:SELECT?", .callback = NULL,}, // Get the current selected correction set ID

    {.pattern = "SENSe:CORRection:FREQuency:START", .callback = NULL,}, // The start frequency of the current cal set
    {.pattern = "SENSe:CORRection:FREQuency:START?", .callback = NULL,}, // Get the start frequency of the current cal set
    {.pattern = "SENSe:CORRection:FREQuency:STOP", .callback = NULL,}, // The stop frequency of the current cal set
    {.pattern = "SENSe:CORRection:FREQuency:STOP?", .callback = NULL,}, // Get the stop frequency of the current cal set

    {.pattern = "SENSe:CORRection:SWEep:POINts", .callback = NULL,}, // The number of points in the current cal set
    {.pattern = "SENSe:CORRection:SWEep:POINts?", .callback = NULL,}, // Get the number of points in the current cal set

    {.pattern = "SENSe:CORRection:COLLect", .callback = NULL,}, // This starts the through callibration (s21)

    // Measurement
    {.pattern = "SENSe:STATus?", .callback = NULL,}, // Checks if the current measurement data is valid

    {.pattern = "SENSe:SELECT", .callback = NULL,}, // Select the active measurement ID
    {.pattern = "SENSe:SELECT?", .callback = NULL,}, // Get the active measurement ID

    {.pattern = "SENSe:FREQuency:START", .callback = NULL,}, // The start frequency of the current measurement set
    {.pattern = "SENSe:FREQuency:START?", .callback = NULL,}, // Get the start frequency of the current measurement set
    {.pattern = "SENSe:FREQuency:STOP", .callback = NULL,}, // The stop frequency of the current measurement set
    {.pattern = "SENSe:FREQuency:STOP?", .callback = NULL,}, // Get the stop frequency of the current measurement set

    {.pattern = "SENSe:SWEep:POINts", .callback = NULL,}, // The number of points in the current measurement set
    {.pattern = "SENSe:SWEep:POINts?", .callback = NULL,}, // Get the number of points in the current measurement set

    {.pattern = "SENSe:REApply", .callback = NULL,}, // Attempts to reapply the calibration data to the current measurement data

    // Measurement and acquisition commands
    {.pattern = "INITiate:IMMediate", .callback = NULL,}, // This starts a single measurement

    // To fetch the current measured data (RDATA/SDATA not supported, FDATA returns polar amplitude and phase in degrees)
    {.pattern = "CALCulate:DATA?", .callback = NULL,},

    SCPI_CMD_LIST_END
};
