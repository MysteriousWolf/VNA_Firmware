//
// Created by matej on 28/08/2024.
//

#include "vna_signaling.h"

scpi_choice_def_t scpi_led_states[] = {
    {"OFF", LED_OFF},
    {"ON", LED_ON},
    {"ERROR", LED_ERROR},
    {"BUSY", LED_BUSY},
    SCPI_CHOICE_LIST_END /* termination of option list */
};

led_states led_state[LED_COUNT] = {};
GPIO_TypeDef* led_port[LED_COUNT] = {LED_1_GPIO_Port, LED_2_GPIO_Port, LED_3_GPIO_Port};
constexpr uint16_t led_pin[LED_COUNT] = {LED_1_Pin, LED_2_Pin, LED_3_Pin};

uint8_t led_thread_stack[SIGNALING_THREAD_STACK_SIZE];
TX_THREAD led_thread_ptr;
uint8_t button_thread_stack[SIGNALING_THREAD_STACK_SIZE];
TX_THREAD button_thread_ptr;

void vna_signaling_init() {
    tx_thread_create(&led_thread_ptr, (char*)"LED Thread", led_thread_entry, 0, led_thread_stack,
        SIGNALING_THREAD_STACK_SIZE, 14, 14, 1, TX_AUTO_START);
    tx_thread_create(&button_thread_ptr, (char*)"Button Thread", button_thread_entry, 0, button_thread_stack,
        SIGNALING_THREAD_STACK_SIZE, 15, 15, 1, TX_AUTO_START);
}

[[noreturn]]
VOID led_thread_entry(ULONG initial_input) {
    int error_blink_counter = 0;
    GPIO_PinState error_blink_state = GPIO_PIN_RESET;
    while (true) {
        for (int i = 0; i < LED_COUNT; i++) {
            switch (led_state[i]) {
            case LED_BUSY:
                HAL_GPIO_TogglePin(led_port[i], led_pin[i]);
                break;
            case LED_ON:
                HAL_GPIO_WritePin(led_port[i], led_pin[i], GPIO_PIN_SET);
                break;
            case LED_OFF:
                HAL_GPIO_WritePin(led_port[i], led_pin[i], GPIO_PIN_RESET);
                break;
            case LED_ERROR:
                HAL_GPIO_WritePin(led_port[i], led_pin[i], error_blink_state);
                break;
            }
        }

        error_blink_counter++;
        if (error_blink_counter > 10) {
            if (error_blink_state == GPIO_PIN_RESET)
                error_blink_state = GPIO_PIN_SET;
            else
                error_blink_state = GPIO_PIN_RESET;
            error_blink_counter = 0;
        }
        tx_thread_sleep(10);
    }
}

int set_led_state(uint8_t led, led_states state) {
    if (led >= LED_COUNT || led < 0)
        return 0;
    led_state[led] = state;
    return 1;
}

// debug global switch states
int sw1 = 0;
int sw2 = 0;
int sw3 = 0;

// debug task to change led states
[[noreturn]]
VOID button_thread_entry(ULONG initial_input) {
    int lednum = 1;
    while (true) {
        /*sw1 = HAL_GPIO_ReadPin(BUTTON_SW1_GPIO_PORT, BUTTON_SW1);
        sw2 = HAL_GPIO_ReadPin(BUTTON_SW2_GPIO_PORT, BUTTON_SW2);
        sw3 = HAL_GPIO_ReadPin(BUTTON_SW3_GPIO_PORT, BUTTON_SW3);
        //if (sw1 == GPIO_PIN_RESET)
        //    led_state[lednum] = LED_BUSY;
        if (sw2 == GPIO_PIN_RESET) {
            led_state[0] = LED_BUSY;
            led_state[1] = LED_ERROR;
            led_state[2] = LED_OFF;
        }
        else if (sw3 == GPIO_PIN_RESET) {
            led_state[0] = LED_ERROR;
            led_state[1] = LED_BUSY;
            led_state[2] = LED_BUSY;
        }
        else {
            led_state[0] = LED_ON;
            led_state[1] = LED_ON;
            led_state[2] = LED_ON;
        }*/
        tx_thread_sleep(20);
    }
}
