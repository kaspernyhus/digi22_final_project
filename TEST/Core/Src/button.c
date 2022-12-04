/**
 * @file user_button.c
 * @author Kasper Nyhus
 * @brief
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "button.h"
#include "print_log.h"

/**
 * @brief Initialize button struct to default values if none given
 *
 * @param button
 */
void button_init(button_t* button, button_config_t* config)
{
    // Save a function pointer to be able to get the pin state (high/low) for debouncing and long press detection
    button->read_pin_state = config->read_pin_cb;
    button->active_state = config->active_state;

    if (config->debounce_ticks != 0) {
        button->debounce_ticks = config->debounce_ticks;
    } else {
        // Default value
        button->debounce_ticks = DEBOUNCE_TICKS;
    }

    if (config->long_press_ticks != 0) {
        button->long_press_ticks = config->long_press_ticks;
    } else {
        // Default value
        button->long_press_ticks = LONG_PRESS_TICKS;
    }

    // Set initial state to WAIT
    button->state = BUTTON_STATE_WAIT;
    button->ticks = 0;
}

/**
 * @brief Register action to perform when button is pressed
 *
 * @param callback function to be called on button press
 * @param press_type short/long press
 */
void button_register_cb(button_t* button, press_action callback, button_press_type_t press_type)
{
    if (press_type == BUTTON_ON_SHORT_PRESS) {
        button->on_press_cb = *callback;
    }
    if (press_type == BUTTON_ON_LONG_PRESS) {
        button->on_long_press_cd = *callback;
    }
}

/**
 * @brief Call to activate button from ISR
 *
 * @param button
 */
void button_activate(button_t* button)
{
    printInfo("BUTTON: activating.");
    button->state = BUTTON_STATE_ACTIVATED;
}

/**
 * @brief Call at an interval. Handle button action. Check for active buttons and debounce, check for long presses
 *
 */
void button_tick(button_t* button)
{
    // If button is in WAIT state, read button pin.
    if (button->state == BUTTON_STATE_WAIT) {
        button_pin_state_t pin_state = button->read_pin_state();
        if (pin_state != button->active_state) {
            // If button pin not active, do nothing
            return;
        } else {
            // Button activated, next debounce.
            button->state = BUTTON_STATE_ACTIVATED;
        }
    } else {
        switch (button->state)
        {

        case BUTTON_STATE_ACTIVATED:
            // Check if debounce ticks reached
            if (button->ticks <= button->debounce_ticks) {
                button->ticks++;
            } else {
                button->ticks = 0;
                // Check if pin is still in active state -> button debounced
                button_pin_state_t pin_state = button->read_pin_state();
                if (pin_state == button->active_state) {
                    // Button confirmed to be pressed
                    button->state = BUTTON_STATE_LONG;
                } else {
                    // A bounce, reset button state
                    button->state = BUTTON_STATE_WAIT;
                }
            }
            break;

        case BUTTON_STATE_LONG: {
                // Check if button is still pressed
                button_pin_state_t pin_state = button->read_pin_state();
                if (pin_state == button->active_state) {
                    // Check if long press time reached
                    if (button->ticks == button->long_press_ticks) {
                        // Check if callback exists
                        if (button->on_long_press_cd != NULL) {
                            button->on_long_press_cd();
                        }
                        button->ticks = 0;
                        button->state = BUTTON_STATE_WAIT_RELEASED;
                    } else {
                        button->ticks++;
                    }
                } else {
                    // Button not active, trigger short press callback
                    // Check if callback exists
                    if (button->on_press_cb != NULL) {
                            button->on_press_cb();
                        }
                    button->ticks = 0;
                    button->state = BUTTON_STATE_WAIT;
                }
            }
            break;

        case BUTTON_STATE_WAIT_RELEASED: {
                // Check if button released
                button_pin_state_t pin_state = button->read_pin_state();
                if (pin_state == button->active_state) {
                    // Still active
                } else {
                    // Reset
                    button->state = BUTTON_STATE_WAIT;
                }
            }
            break;

        default:
            break;
        }
    }
}

