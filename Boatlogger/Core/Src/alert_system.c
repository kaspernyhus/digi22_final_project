/**
 * @file alert.c
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-12-01
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "alert_system.h"
#include <string.h>
#include "print_log.h"

static alert_t alerts[MAX_ALERTS];
static uint8_t alerts_registered = 0;
static alert_state_t current_alert_state = ALERT_HIGH;

TIM_HandleTypeDef* _tim;

// TODO: return if event caused state change, to pass to LCD auto show ALERTS page

/**
 * @brief Change system wide alert state
 *
 * @param state requested new alert state
 */
static bool change_state(alert_state_t state) {
    bool state_changed = false;
    // Only change state if requested state different from current state
    if (current_alert_state != state) {
        state_changed = true;
        current_alert_state = state;

        switch (current_alert_state)
        {
        case ALERT_NORMAL:
            __HAL_TIM_SET_AUTORELOAD(_tim, NORMAL_RATE);
            __HAL_TIM_SET_COUNTER(_tim, 0);
            break;
        case ALERT_LOW:
            __HAL_TIM_SET_AUTORELOAD(_tim, LOW_ALERT_RATE);
            __HAL_TIM_SET_COUNTER(_tim, 0);
            break;
        case ALERT_HIGH:
            __HAL_TIM_SET_AUTORELOAD(_tim, HIGH_ALERT_RATE);
            __HAL_TIM_SET_COUNTER(_tim, 0);
            break;
        default:
            __HAL_TIM_SET_AUTORELOAD(_tim, NORMAL_RATE);
            __HAL_TIM_SET_COUNTER(_tim, 0);
            break;
        }

        // Print alert levels
        for (int i=0; i<alerts_registered; i++) {
            char buf[100];
            sprintf(buf, "%s: %s", alerts[i].name, alert_state_name[alerts[i].alert_state]);
            printWarning(buf);
        }
    }
    // Return if state has changed
    return state_changed;
}

/**
 * @brief Look for raised alert levels in registered alert modules
 *
 */
static bool evaluate_alert_state(void)
{
    bool state_changed = false;
    // Look through registered alerts to find raised alert levels, highest set sytem wide alert level
    alert_state_t highest_alert = ALERT_NORMAL;
    for (int i=0; i<alerts_registered; i++) {
        if (alerts[i].alert_state != ALERT_NORMAL) {
            if (highest_alert < alerts[i].alert_state) {
                highest_alert = alerts[i].alert_state;
            }
        }
    }
    if (current_alert_state != highest_alert) {
        state_changed = change_state(highest_alert);
    }
    return state_changed;
}

/**
 * @brief Initialize alert system
 *
 * @param timer_handle handle to timer to control LED flash rate
 */
void alert_system_init(TIM_HandleTypeDef* timer_handle)
{
    _tim = timer_handle;
    evaluate_alert_state();
}

/**
 * @brief Register an alert to the alert system
 *
 * @param alert_type
 * @param low_thresshold
 * @param high_thresshold
 */
void alert_system_register(alert_type_t alert_type, char* name, alert_threshold_t threshold_type, float low_threshold, float high_threshold)
{
    alert_t new_alert = {
        .alert_type = alert_type,
        .threshold_type = threshold_type,
        .low_thresshold = low_threshold,
        .high_thresshold = high_threshold,
        .alert_state = ALERT_NORMAL,
        .name = name
    };
    memcpy(&alerts[alerts_registered], &new_alert, sizeof(alert_t));
    char buf[100];
    sprintf(buf, "%s alert registered.", alerts[alerts_registered].name);
    printInfo(buf);
    alerts_registered++;
}

/**
 * @brief Checks if a value is consideret a hazard (above or below a set threshold)
 *
 * @param value float value of measured data
 * @param type type of measurement
 */
bool alert_system_check(float value, alert_type_t type, alert_state_t* state)
{
    // Look through registered alerts for matching type
    for (int i=0; i<alerts_registered; i++) {
        if (type == alerts[i].alert_type) {
            switch (alerts[i].threshold_type)
            {
            case ALERT_ABOVE_THRESHOLD:
                if (value < alerts[i].low_thresshold) {
                    // No alert
                    alerts[i].alert_state = ALERT_NORMAL;
                } else if (value > alerts[i].high_thresshold) {
                    // High alert
                    alerts[i].alert_state = ALERT_HIGH;
                } else {
                    // low_thresshold <= value < high_thresshold
                    alerts[i].alert_state = ALERT_LOW;
                }
                break;

            case ALERT_BELOW_THRESHOLD:
                if (value > alerts[i].high_thresshold) {
                    // No alert
                    alerts[i].alert_state = ALERT_NORMAL;
                } else if (value < alerts[i].low_thresshold) {
                    // High alert
                    alerts[i].alert_state = ALERT_HIGH;
                } else {
                    // low_thresshold <= value < high_thresshold
                    alerts[i].alert_state = ALERT_LOW;
                }
                break;
            default:
                break;
            }
            *state = alerts[i].alert_state;
        }
    }
    // Return if state has changed
    return evaluate_alert_state();
}

/**
 * @brief Return current alert level
 *
 * @return alert_state_t
 */
alert_state_t alert_system_alert_level(void)
{
    return current_alert_state;
}
