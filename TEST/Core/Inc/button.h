/**
 * @file user_button.h
 * @author Kasper Nyhus
 * @brief
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include <stdint.h>

typedef enum {
    BUTTON_STATE_WAIT,
    BUTTON_STATE_ACTIVATED,
    BUTTON_STATE_DEBOUNCED,
    BUTTON_STATE_LONG,
    BUTTON_STATE_WAIT_RELEASED
} button_state_t;

typedef enum {
    BUTTON_PIN_STATE_LOW,
    BUTTON_PIN_STATE_HIGH
} button_pin_state_t;

typedef enum {
    BUTTON_ON_SHORT_PRESS,
    BUTTON_ON_LONG_PRESS
} button_press_type_t;

typedef button_pin_state_t (*read_button_pin)(void);
typedef void (*press_action)(void);

typedef struct {
    button_state_t state;
    button_pin_state_t active_state;
    uint16_t ticks;
    uint8_t debounce_ticks;
    uint16_t long_press_ticks;
    read_button_pin read_pin_state;
    press_action on_press_cb;
    press_action on_long_press_cd;
} button_t;

typedef struct {
    button_pin_state_t active_state;
    uint8_t debounce_ticks;
    uint16_t long_press_ticks;
    read_button_pin read_pin_cb;
} button_config_t;

void button_init(button_t* button, button_config_t* config);
void button_register_cb(button_t* button, press_action callback, button_press_type_t press_type);
void button_activate(button_t* button);
void button_tick(button_t* button);
