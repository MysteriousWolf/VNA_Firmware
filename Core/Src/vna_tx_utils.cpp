//
// Created by matej on 01/09/2024.
//

#include "../Inc/vna_tx_utils.h"

/**
 * Initialize the utilities. Currently, start the microsecond timer.
 */
void init_utils() {
    HAL_TIM_Base_Start(&US_TIMER);
}

/**
 * Delay in microseconds (requires a timer with a 1 MHz frequency)
 * @param us The number of microseconds to delay
 */
void delay_us(const int us) {
    // Get the current timer value
    const uint32_t start = __HAL_TIM_GET_COUNTER(&US_TIMER);
    while ((__HAL_TIM_GET_COUNTER(&US_TIMER) - start) < us); // Wait until enough microseconds have passed
}

/**
 * In range check 64-bit
 * @param value The value to check
 * @param min The minimum value
 * @param max The maximum value
 * @return Whether the value is in the range
 */
bool in_range64(const uint64_t value, const uint64_t min, const uint64_t max) {
    return value >= min && value <= max;
}

/**
 * In range check 32-bit
 * @param value The value to check
 * @param min The minimum value
 * @param max The maximum value
 * @return Whether the value is in the range
 */
bool in_range(const uint32_t value, const uint32_t min, const uint32_t max) {
    return value >= min && value <= max;
}

/**
 * Limit a value to a range
 * @param value The value to limit
 * @param min The minimum value
 * @param max The maximum value
 * @return The limited value
 */
uint32_t limit(const uint32_t value, const uint32_t min, const uint32_t max) {
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

/**
 * Append a character array to another character array efficiently
 * @param destination The destination character array (needs to be zero terminated and have enough space)
 * @param destinationSize The size of the destination character array
 * @param source The source character array
 * @return The number of characters appended
 */
size_t appendCharArray(char* destination, const size_t destinationSize, const char* source) {
    // Find the length of the destination string (up to destinationSize - 1)
    int destLength = 0;
    while (destLength < destinationSize - 1 && destination[destLength] != '\0') {
        destLength++;
    }

    // Append source string to destination string
    int i = 0;
    while (destLength < destinationSize - 1 && source[i] != '\0') {
        destination[destLength++] = source[i++];
    }

    // Ensure the destination string is null-terminated
    destination[destLength] = '\0';
    return destLength;
}

/**
 * Convert an integer to a character array
 * @param value The integer value to convert
 * @param buffer The buffer to store the string
 * @param bufferSize The size of the buffer
 * @param terminate Whether to null-terminate the string
 * @return The number of characters written to the buffer
 */
size_t intToCharArray(int value, char* buffer, const size_t bufferSize, const bool terminate) {
    // Calculate the length required for the integer string
    int length = 0;
    int temp = value;

    // Handle zero case
    if (value == 0) {
        length = 1; // "0"
    } else {
        // Account for negative sign
        if (value < 0) {
            length++;
            temp = -temp; // Make positive for digit counting
        }

        // Count digits
        while (temp > 0) {
            length++;
            temp /= 10;
        }
    }

    // Add one more for the null terminator if required
    const int requiredSize = length + (terminate ? 1 : 0);

    // Check if buffer is large enough
    if (bufferSize < requiredSize) {
        // If not enough space, return that 0 characters were written
        return 0;
    }

    // Convert the integer to a string, starting from the end of the buffer
    int index = length;
    if (terminate) {
        buffer[length] = '\0'; // Null-terminate if required
    }

    // Handle zero case
    if (value == 0) {
        buffer[--index] = '0';
    } else {
        int isNegative = 0;
        if (value < 0) {
            isNegative = 1;
            value = -value;
        }

        // Convert each digit to a character
        while (value > 0) {
            buffer[--index] = (value % 10) + '0';
            value /= 10;
        }

        // Add the negative sign if necessary
        if (isNegative) {
            buffer[--index] = '-';
        }
    }
    return length;
}

/**
 * Get the next message type from a queue
 * @param char_queue The threadx queue to get the message type from
 * @return The message type
 */
vna_msg_type get_next_msg_type(TX_QUEUE* char_queue) {
    vna_msg_type msg_type;
    if (const UINT status = tx_queue_receive(char_queue, &msg_type, TX_WAIT_FOREVER); status != TX_SUCCESS)
        return VNA_MSG_NONE;
    return msg_type;
}

/**
 * Get the next message length from a queue
 * @param char_queue The threadx queue to get the message length from
 * @return The message length
 */
size_t get_next_msg_len(TX_QUEUE* char_queue) {
    ULONG len = 0;
    // Receive the length first
    if (const UINT status = tx_queue_receive(char_queue, &len, TX_WAIT_FOREVER); status != TX_SUCCESS) {
        // Handle error
        return 0;
    }
    return len;
}

/**
 * Send string data with length information to a queue
 * @param char_queue The threadx queue to send data to
 * @param data The data to send
 * @param len The length of the data
 * @return The actual number of characters sent
 */
size_t send_text_to_queue(TX_QUEUE* char_queue, const char* data, const size_t len) {
    ULONG packed_data;
    size_t i;
    const size_t num_chunks = len / 4; // Number of full ULONG chunks
    const size_t remaining_chars = len % 4; // Remaining characters after full chunks
    size_t sent_chars = 0;

    // Send the message type first
    ULONG msg_type = VNA_MSG_TEXT;
    UINT status = tx_queue_send(char_queue, &msg_type, TX_WAIT_FOREVER);
    if (status != TX_SUCCESS) {
        // Handle error
        return sent_chars;
    }

    // Send the length of the data second
    ULONG length = len;
    status = tx_queue_send(char_queue, &length, TX_WAIT_FOREVER);
    if (status != TX_SUCCESS) {
        // Handle error
        return sent_chars;
    }

    // Send the full ULONG chunks
    for (i = 0; i < num_chunks; i++) {
        packed_data = 0;
        packed_data |= (static_cast<ULONG>(data[i * 4 + 0]) << 24);
        packed_data |= (static_cast<ULONG>(data[i * 4 + 1]) << 16);
        packed_data |= (static_cast<ULONG>(data[i * 4 + 2]) << 8);
        packed_data |= static_cast<ULONG>(data[i * 4 + 3]);

        status = tx_queue_send(char_queue, &packed_data, TX_WAIT_FOREVER);
        if (status != TX_SUCCESS) {
            // Handle error
            return sent_chars;
        }
        sent_chars += 4;
    }

    // Handle the remaining characters (if any)
    if (remaining_chars > 0) {
        packed_data = 0;
        for (i = 0; i < remaining_chars; i++) {
            packed_data |= (static_cast<ULONG>(data[num_chunks * 4 + i]) << (24 - i * 8));
        }

        status = tx_queue_send(char_queue, &packed_data, TX_WAIT_FOREVER);
        if (status != TX_SUCCESS) {
            // Handle error
            return sent_chars;
        }
        sent_chars += remaining_chars;
    }

    return sent_chars;
}

/**
 * Receive string data from a queue
 * @param char_queue The threadx queue to receive data from
 * @param buffer The buffer to store the received data
 * @param len The length specified in the header of the received data
 * @return The actual number of characters received
 */
size_t receive_text_from_queue(TX_QUEUE* char_queue, char* buffer, const size_t len) {
    UINT status = 0;
    ULONG packed_data;
    size_t i;
    size_t received_chars = 0;

    const size_t num_chunks = len / 4;
    const size_t remaining_chars = len % 4;

    // Receive the full ULONG chunks
    for (i = 0; i < num_chunks; i++) {
        status = tx_queue_receive(char_queue, &packed_data, TX_WAIT_FOREVER);
        if (status != TX_SUCCESS) {
            // Handle error
            return received_chars;
        }
        buffer[i * 4 + 0] = static_cast<char>((packed_data >> 24) & 0xFF);
        buffer[i * 4 + 1] = static_cast<char>((packed_data >> 16) & 0xFF);
        buffer[i * 4 + 2] = static_cast<char>((packed_data >> 8) & 0xFF);
        buffer[i * 4 + 3] = static_cast<char>(packed_data & 0xFF);
        received_chars += 4;
    }

    // Handle remaining characters
    if (remaining_chars > 0) {
        status = tx_queue_receive(char_queue, &packed_data, TX_WAIT_FOREVER);
        if (status != TX_SUCCESS) {
            // Handle error
            return received_chars;
        }
        for (i = 0; i < remaining_chars; i++) {
            buffer[num_chunks * 4 + i] = static_cast<char>((packed_data >> (24 - i * 8)) & 0xFF);
        }
        received_chars += remaining_chars;
    }

    return received_chars;
}

size_t send_meas_data_to_queue(TX_QUEUE* char_queue) {
    // Send the message type first
    ULONG msg_type = VNA_MSG_MEAS_DATA;
    UINT status = tx_queue_send(char_queue, &msg_type, TX_WAIT_FOREVER);
    if (status != TX_SUCCESS) {
        // Handle error
        return 0;
    }

    // Send the length of the data second (0 as a placeholder)
    ULONG length = 0;
    status = tx_queue_send(char_queue, &length, TX_WAIT_FOREVER);
    if (status != TX_SUCCESS) {
        // Handle error
        return 0;
    }

    return sizeof(char*); // pointer message hack length
}
