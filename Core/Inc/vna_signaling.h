//
// Created by matej on 28/08/2024.
//

#ifndef SIGNALING_H
#define SIGNALING_H

#include "main.h"
#include "app_threadx.h"

#include "scpi/scpi.h"

#define LED_COUNT 3

// This needs to be visible from C code
#ifdef __cplusplus
extern "C" {
#endif

typedef enum led_states {
    LED_OFF = 0U,
    LED_BUSY,
    LED_ERROR,
    LED_ON,
} led_states;

extern scpi_choice_def_t scpi_led_states[];

void vna_signaling_init();
int set_led_state(uint8_t led, led_states state);

#ifdef __cplusplus
}
#endif

[[noreturn]] VOID led_thread_entry(ULONG initial_input);

[[noreturn]] VOID button_thread_entry(ULONG initial_input);

#endif //SIGNALING_H
