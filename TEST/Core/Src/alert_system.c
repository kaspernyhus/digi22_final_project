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

static void change_state(alert_state_t new_state)
{
    if (new_state != current_alert_state) {
        current_alert_state = new_state;
        char buf[100];
        sprintf(buf, "New alert state: %s", alert_state_name[current_alert_state]);
        printInfo(buf);

        // Print info on all alerts
        for (int i=0; i<alerts_registered; i++) {
            if (alerts[i].alert_state != ALERT_NORMAL) {
                sprintf(buf,"%s: %s", alerts[i].name, alert_state_name[alerts[i].alert_state]);
                printWarning(buf);
            }
        }

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
    }
}

void alert_system_init(TIM_HandleTypeDef* timer_handle)
{
    _tim = timer_handle;
    change_state(ALERT_HIGH);
}

/**
 * @brief Register an alert to the alert system
 *
 * @param alert_type
 * @param low_thresshold
 * @param high_thresshold
 */
void alert_system_register(alert_type_t alert_type, char* name, float low_thresshold, float high_thresshold)
{
    alert_t new_alert = {
        .alert_type = alert_type,
        .low_thresshold = low_thresshold,
        .high_thresshold = high_thresshold,
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
 * @brief Checks if a value is consideret a hazard
 *
 * @param value float value of measured data
 * @param type type of measurement
 */
void alert_system_check(float value, alert_type_t type)
{

    for (int i=0; i<alerts_registered; i++) {
        if (type == alerts[i].alert_type) {
            // Check values
            if (value < alerts[i].low_thresshold) {
                // No alert
                return;
            } else if (value > alerts[i].high_thresshold) {
                // High alert
                alerts[i].alert_state = ALERT_HIGH;
                change_state(ALERT_HIGH);
            } else {
                // low_thresshold < value < high_thresshold
                alerts[i].alert_state = ALERT_LOW;
                change_state(ALERT_LOW);
            }
        }
    }
}
