//
// Created by matej on 30/08/2024.
//

#include "vna_cmd_engine.h"

#include "vna_stuw81300.h"

// SCPI frequency units with their multipliers
scpi_choice_def_t scpi_choice_frequency_units[] = {
    {"Hz", 1},
    {"kHz", 1000},
    {"MHz", 1000000},
    {"GHz", 1000000000},
    SCPI_CHOICE_LIST_END
};

uint8_t out_queue_stack[QUEUE_STACK_SIZE];
TX_QUEUE out_queue;

scpi_t scpi_context;

char scpi_input_buffer[SCPI_INPUT_BUFFER_LENGTH];
scpi_error_t scpi_error_queue_data[SCPI_ERROR_QUEUE_SIZE];

// Init global settings
bool echo_enabled = ECHO_DEFAULT;

// Initialize the command engine queue and parser
void vna_init_cmd_engine() {
    // Initialize the message queues
    tx_queue_create(&out_queue, (char*)"SCPI Out Queue", 1, out_queue_stack, QUEUE_STACK_SIZE);

    // Initialize the SCPI parser
    SCPI_Init(&scpi_context, scpi_commands, &scpi_interface, scpi_units_def,
              SCPI_IDN1, SCPI_IDN2, SCPI_IDN3, SCPI_IDN4,
              scpi_input_buffer, SCPI_INPUT_BUFFER_LENGTH,
              scpi_error_queue_data, SCPI_ERROR_QUEUE_SIZE);
}

void vna_process_command(const UCHAR* buffer, const ULONG length) {
    // Echo the command if enabled
    if (echo_enabled)
        send_text_to_queue(&out_queue, (const char*)buffer, length);
    // Process the command
    SCPI_Input(&scpi_context, (const char*)buffer, static_cast<int>(length));
}

/* SCPI core functions */
size_t SCPI_Write(scpi_t* context, const char* data, size_t len) {
    (void)context; // Unused

    // Send the data to the message queue in packed form
    return send_text_to_queue(&out_queue, data, len);
}

int SCPI_Error(scpi_t* context, int_fast16_t err) {
    (void)context; // Unused

    char err_str[17] = "ERROR: ";
    char err_code[7];

    // Build the error message
    intToCharArray(err, err_code, 7, true);
    appendCharArray(err_str, 16, err_code);
    size_t len = appendCharArray(err_str, 16, "\r\n");

    // Send the error to the message queue in packed form
    return static_cast<int>(send_text_to_queue(&out_queue, err_str, len));
}

scpi_result_t SCPI_Control(scpi_t* context, scpi_ctrl_name_t ctrl, scpi_reg_val_t val) {
    (void)context; // Unused
    return SCPI_RES_OK;
}

scpi_result_t SCPI_Flush(scpi_t* context) {
    (void)context; // Unused
    return SCPI_RES_OK;
}

scpi_result_t SCPI_Reset(scpi_t* context) {
    (void)context; // Unused
    return SCPI_RES_OK;
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
    SCPI_ResultInt32(context, 0);
    return SCPI_RES_OK;
}

// Extended SCPI commands
static scpi_result_t VNA_CoreEcho(scpi_t* context) {
    bool echo_param;

    /* read first parameter if present */
    if (SCPI_ParamBool(context, &echo_param, false)) {
        echo_enabled = echo_param;
    } else {
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
        // Report an error code if parsing fails
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
        // Report an error code if out of range
        SCPI_ResultInt64(context, -2);
        return SCPI_RES_ERR;
    }

    // Set the STUW81300 frequency directly using the driver
    const auto actual_frequency = static_cast<int64_t>(set_frequency_STUW81300(frequency));

    // Report the actual frequency set back to the user
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
    } else {
        // If none is provided, toggle the mute state
        mute_param = !is_muted_STUW81300();
        mute_STUW81300(mute_param);
    }

    // Check if it's actually muted and report the state back to the user
    SCPI_ResultBool(context, is_muted_STUW81300());
    return SCPI_RES_OK;
}

/* VNA SCPI commands*/
// Core commands
static scpi_result_t scpi_vco_stat_q(scpi_t* context) {
    SCPI_ResultInt32(context, vna_get_status());
    return SCPI_RES_OK;
}

// Calibration related functions
static scpi_result_t scpi_vco_corr_en(scpi_t* context) {
    bool correction_en_param;

    // Read the state parameter
    if (SCPI_ParamBool(context, &correction_en_param, false)) {
        vna_enable_calib(correction_en_param);
    } else {
        // If none is provided, toggle the correction state
        correction_en_param = !vna_is_calib_enabled();
        vna_enable_calib(correction_en_param);
    }

    // Report the actual state
    SCPI_ResultBool(context, vna_is_calib_enabled());

    return SCPI_RES_OK;
}

static scpi_result_t scpi_vco_corr_en_q(scpi_t* context) {
    // Report the state
    SCPI_ResultBool(context, vna_is_calib_enabled());

    return SCPI_RES_OK;
}

static scpi_result_t scpi_vco_corr_stat_q(scpi_t* context) {
    // Report whether the correction data is valid
    SCPI_ResultBool(context, vna_is_calib_valid());

    return SCPI_RES_OK;
}

static scpi_result_t scpi_vco_corr_val_q(scpi_t* context) {
    // Report whether the current correction data is valid for the currently active measurement data
    SCPI_ResultBool(context, vna_is_calib_valid_for_meas(vna_get_active_calib_meta()));

    return SCPI_RES_OK;
}

static scpi_result_t scpi_vco_corr_sel(scpi_t* context) {
    uint32_t calib_id;

    // Get the calibration ID from the command
    if (!SCPI_ParamUInt32(context, &calib_id, true)) {
        return SCPI_RES_ERR;
    }

    // Set the active calibration data set
    if (const int32_t status = vna_set_active_calib(calib_id); status <= -10) {
        // Report the error code with success - the calibration set was changed but could not be applied
        SCPI_ResultInt32(context, status);
        return SCPI_RES_OK;
    } else if (status < 0)
        return SCPI_RES_ERR;

    // If everything is OK, report the active calibration ID
    SCPI_ResultUInt32(context, calib_id);

    return SCPI_RES_OK;
}

static scpi_result_t scpi_vco_corr_sel_q(scpi_t* context) {
    // Report the active calibration ID
    SCPI_ResultInt32(context, vna_get_active_calib());

    return SCPI_RES_OK;
}

static scpi_result_t scpi_vco_corr_freq_start(scpi_t* context) {
    uint64_t frequency;
    int32_t multiplier;

    // Parse the numeric part of the command
    if (!SCPI_ParamUInt64(context, &frequency, true)) {
        // Report an error code if parsing fails
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
        // Report an error code if out of range
        SCPI_ResultInt64(context, -2);
        return SCPI_RES_ERR;
    }

    // Set the calibration start frequency
    const int32_t status = vna_set_calib_start_frequency(frequency);

    if (status < 0) {
        SCPI_ResultInt64(context, status * 10); // Report the error code x10 to differentiate from other errors
        return SCPI_RES_ERR;
    }

    // Report the actual frequency set back to the user
    SCPI_ResultUInt64(context, frequency);
    return SCPI_RES_OK;
}

static scpi_result_t scpi_vco_corr_freq_start_q(scpi_t* context) {
    // Report the start frequency
    SCPI_ResultUInt64(context, vna_get_calib_start_frequency());

    return SCPI_RES_OK;
}

static scpi_result_t scpi_vco_corr_freq_stop(scpi_t* context) {
    uint64_t frequency;
    int32_t multiplier;

    // Parse the numeric part of the command
    if (!SCPI_ParamUInt64(context, &frequency, true)) {
        // Report an error code if parsing fails
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
        // Report an error code if out of range
        SCPI_ResultInt64(context, -2);
        return SCPI_RES_ERR;
    }

    // Set the calibration stop frequency
    const int32_t status = vna_set_calib_stop_frequency(frequency);

    if (status < 0) {
        SCPI_ResultInt64(context, status * 10); // Report the error code x10 to differentiate from other errors
        return SCPI_RES_ERR;
    }

    // Report the actual frequency set back to the user
    SCPI_ResultUInt64(context, frequency);

    return SCPI_RES_OK;
}

static scpi_result_t scpi_vco_corr_freq_stop_q(scpi_t* context) {
    // Report the stop frequency
    SCPI_ResultUInt64(context, vna_get_calib_stop_frequency());

    return SCPI_RES_OK;
}

static scpi_result_t scpi_vco_corr_points(scpi_t* context) {
    uint32_t point_count;

    // Get the calibration ID from the command
    if (!SCPI_ParamUInt32(context, &point_count, true)) {
        return SCPI_RES_ERR;
    }

    // Set the active calibration data set
    if (const int32_t status = vna_set_calib_point_count(point_count); status < 0) {
        // Report the error code
        SCPI_ResultInt32(context, status);
        return SCPI_RES_ERR;
    }

    // If everything is OK, report the active calibration ID
    SCPI_ResultUInt32(context, point_count);

    return SCPI_RES_OK;
}

static scpi_result_t scpi_vco_corr_points_q(scpi_t* context) {
    // Report the number of points
    SCPI_ResultUInt32(context, vna_get_calib_point_count());

    return SCPI_RES_OK;
}

static scpi_result_t scpi_vco_corr_coll(scpi_t* context) {
    // Start the calibration collection
    if (const int32_t status = vna_request_job(VNA_JOB_CALIBRATE); status < 0) {
        // Report the error code
        SCPI_ResultInt32(context, status);
        return SCPI_RES_ERR;
    }

    // If everything is OK, report the active calibration ID
    SCPI_ResultUInt32(context, vna_get_active_calib());

    return SCPI_RES_OK;
}

static scpi_result_t scpi_vco_meas_stat_q(scpi_t* context) {
    // Report whether the measurement data is valid
    SCPI_ResultBool(context, vna_is_meas_valid());

    return SCPI_RES_OK;
}

static scpi_result_t scpi_vco_meas_sel(scpi_t* context) {
    uint32_t calib_id;

    // Get the measurement ID from the command
    if (!SCPI_ParamUInt32(context, &calib_id, true)) {
        return SCPI_RES_ERR;
    }

    // Set the active measurement data set
    if (const int32_t status = vna_set_active_meas(calib_id); status <= -10) {
        // Report the error code with success - the calibration set was changed but could not be applied
        SCPI_ResultInt32(context, status);
        return SCPI_RES_OK;
    } else if (status < 0)
        return SCPI_RES_ERR;

    // If everything is OK, report the active calibration ID
    SCPI_ResultUInt32(context, calib_id);

    return SCPI_RES_OK;
}

static scpi_result_t scpi_vco_meas_sel_q(scpi_t* context) {
    // Report the active measurement ID
    SCPI_ResultInt32(context, vna_get_active_meas());

    return SCPI_RES_OK;
}

static scpi_result_t scpi_vco_meas_freq_start(scpi_t* context) {
    uint64_t frequency;
    int32_t multiplier;

    // Parse the numeric part of the command
    if (!SCPI_ParamUInt64(context, &frequency, true)) {
        // Report an error code if parsing fails
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
        // Report an error code if out of range
        SCPI_ResultInt64(context, -2);
        return SCPI_RES_ERR;
    }

    // Set the measurement start frequency
    const int32_t status = vna_set_meas_start_frequency(frequency);

    if (status < 0) {
        SCPI_ResultInt64(context, status * 10); // Report the error code x10 to differentiate from other errors
        return SCPI_RES_ERR;
    }

    // Report the actual frequency set back to the user
    SCPI_ResultUInt64(context, frequency);

    return SCPI_RES_OK;
}

static scpi_result_t scpi_vco_meas_freq_start_q(scpi_t* context) {
    // Report the start frequency
    SCPI_ResultUInt64(context, vna_get_calib_start_frequency());

    return SCPI_RES_OK;
}

static scpi_result_t scpi_vco_meas_freq_stop(scpi_t* context) {
    uint64_t frequency;
    int32_t multiplier;

    // Parse the numeric part of the command
    if (!SCPI_ParamUInt64(context, &frequency, true)) {
        // Report an error code if parsing fails
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
        // Report an error code if out of range
        SCPI_ResultInt64(context, -2);
        return SCPI_RES_ERR;
    }

    // Set the measurement stop frequency
    const int32_t status = vna_set_meas_stop_frequency(frequency);

    if (status < 0) {
        SCPI_ResultInt64(context, status * 10); // Report the error code x10 to differentiate from other errors
        return SCPI_RES_ERR;
    }

    // Report the actual frequency set back to the user
    SCPI_ResultUInt64(context, frequency);
    return SCPI_RES_OK;
}

static scpi_result_t scpi_vco_meas_freq_stop_q(scpi_t* context) {
    // Report the stop frequency
    SCPI_ResultUInt64(context, vna_get_calib_stop_frequency());

    return SCPI_RES_OK;
}

static scpi_result_t scpi_vco_meas_points(scpi_t* context) {
    uint32_t point_count;

    // Get the measurement ID from the command
    if (!SCPI_ParamUInt32(context, &point_count, true)) {
        return SCPI_RES_ERR;
    }

    // Set the active measurement data set
    if (const int32_t status = vna_set_meas_point_count(point_count); status < 0) {
        // Report the error code
        SCPI_ResultInt32(context, status);
        return SCPI_RES_ERR;
    }

    // If everything is OK, report the active calibration ID
    SCPI_ResultUInt32(context, point_count);

    return SCPI_RES_OK;
}

static scpi_result_t scpi_vco_meas_points_q(scpi_t* context) {
    // Report the number of points
    SCPI_ResultUInt32(context, vna_get_meas_point_count());

    return SCPI_RES_OK;
}

static scpi_result_t scpi_vco_meas_reapply_calib(scpi_t* context) {
    // Start the corrected measured data calculation
    if (const int32_t status = vna_request_job(VNA_JOB_APPLY_CORRECTION); status < 0) {
        // Report the error code
        SCPI_ResultInt32(context, status);
        return SCPI_RES_ERR;
    }

    // If everything is OK, report the active calibration ID
    SCPI_ResultUInt32(context, vna_get_active_meas());

    return SCPI_RES_OK;
}

static scpi_result_t scpi_vco_meas_init(scpi_t* context) {
    // Start the measurement collection
    if (const int32_t status = vna_request_job(VNA_JOB_MEASURE); status < 0) {
        // Report the error code
        SCPI_ResultInt32(context, status);
        return SCPI_RES_ERR;
    }

    // If everything is OK, report the active measurement ID
    SCPI_ResultUInt32(context, vna_get_active_meas());

    return SCPI_RES_OK;
}

static scpi_result_t scpi_vco_calc_data_q(scpi_t* context) {
    // Copy the data to the write protected output buffer
    meas_data_outgoing = vna_is_calib_enabled() ? meas_data_corrected : meas_data[vna_get_active_meas()];

    // Send the message to start the data transfer
    send_meas_data_to_queue(&out_queue);

    // Suspend reading until the data is fully sent
    ULONG actual_flags;
    tx_event_flags_get(&measurement_event_flags, READOUT_EVENT_FLAG, TX_AND_CLEAR, &actual_flags, TX_WAIT_FOREVER);

    // Send dummy text to the queue to finish the message with standard scpi response
    SCPI_ResultText(context, "");

    return SCPI_RES_OK;
}

// Core SCPI structs
scpi_interface_t scpi_interface = {
    .error = SCPI_Error,
    .write = SCPI_Write,
    .control = SCPI_Control,
    .flush = SCPI_Flush,
    .reset = SCPI_Reset,
};

// VNA Command list structure
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
    // Core commands
    {.pattern = "STATus:OPERation?", .callback = scpi_vco_stat_q,}, // To get the current vna core state

    // Calibration
    {.pattern = "SENSe:CORRection:ENable", .callback = scpi_vco_corr_en,}, // Enable correction data
    {.pattern = "SENSe:CORRection:ENable?", .callback = scpi_vco_corr_en_q,}, // Check if meas correction is enabled

    // Check if the correction data is valid
    {.pattern = "SENSe:CORRection:STATus?", .callback = scpi_vco_corr_stat_q,},
    // Checks if the current measurement data matches the current correction data
    {.pattern = "SENSe:CORRection:VALid?", .callback = scpi_vco_corr_val_q,},

    // Select the active calibration ID
    {.pattern = "SENSe:CORRection:SELect", .callback = scpi_vco_corr_sel,},
    // Get the current selected correction set ID
    {.pattern = "SENSe:CORRection:SELect?", .callback = scpi_vco_corr_sel_q,},

    // The start frequency of the current cal set
    {.pattern = "SENSe:CORRection:FREQuency:START", .callback = scpi_vco_corr_freq_start,},
    // Get the start frequency of the current cal set
    {.pattern = "SENSe:CORRection:FREQuency:START?", .callback = scpi_vco_corr_freq_start_q,},
    // The stop frequency of the current cal set
    {.pattern = "SENSe:CORRection:FREQuency:STOP", .callback = scpi_vco_corr_freq_stop,},
    // Get the stop frequency of the current cal set
    {.pattern = "SENSe:CORRection:FREQuency:STOP?", .callback = scpi_vco_corr_freq_stop_q,},

    // The number of points in the current cal set
    {.pattern = "SENSe:CORRection:SWEep:POINts", .callback = scpi_vco_corr_points,},
    // Get the number of points in the current cal set
    {.pattern = "SENSe:CORRection:SWEep:POINts?", .callback = scpi_vco_corr_points_q,},

    // This starts the through calibration (s21)
    {.pattern = "SENSe:CORRection:COLLect", .callback = scpi_vco_corr_coll,},

    // Measurement
    {.pattern = "SENSe:STATus?", .callback = scpi_vco_meas_stat_q,}, // Checks if the current measurement data is valid

    {.pattern = "SENSe:SELect", .callback = scpi_vco_meas_sel,}, // Select the active measurement ID
    {.pattern = "SENSe:SELect?", .callback = scpi_vco_meas_sel_q,}, // Get the active measurement ID

    // The start frequency of the current measurement set
    {.pattern = "SENSe:FREQuency:START", .callback = scpi_vco_meas_freq_start,},
    // Get the start frequency of the current measurement set
    {.pattern = "SENSe:FREQuency:START?", .callback = scpi_vco_meas_freq_start_q,},
    // The stop frequency of the current measurement set
    {.pattern = "SENSe:FREQuency:STOP", .callback = scpi_vco_meas_freq_stop,},
    // Get the stop frequency of the current measurement set
    {.pattern = "SENSe:FREQuency:STOP?", .callback = scpi_vco_meas_freq_stop_q,},

    // The number of points in the current measurement set
    {.pattern = "SENSe:SWEep:POINts", .callback = scpi_vco_meas_points,},
    // Get the number of points in the current measurement set
    {.pattern = "SENSe:SWEep:POINts?", .callback = scpi_vco_meas_points_q,},

    // Attempts to reapply the calibration data to the current measurement data
    {.pattern = "SENSe:REApply", .callback = scpi_vco_meas_reapply_calib,},

    // Measurement and acquisition commands
    {.pattern = "INITiate:IMMediate", .callback = scpi_vco_meas_init,}, // This starts a single measurement

    // To fetch the current measured data
    // RDATA/SDATA not supported, always returns FDATA returns polar amplitude and phase in degrees
    // Returns interchanging phase and amplitude separated by a comma
    {.pattern = "CALCulate:DATA?", .callback = scpi_vco_calc_data_q,},

    SCPI_CMD_LIST_END
};
