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

#include "alert.h"
#include "log.h"

alert_state_t current_alert_state = ALERT_HIGH;
TIM_HandleTypeDef* _tim;

static void change_state(alert_state_t new_state)
{
    if (new_state != current_alert_state) {
        current_alert_state = new_state;
        char buf[35];
        sprintf(buf, "New alert state: %s", alert_state_name[current_alert_state]);
        LOG(buf);

        switch (current_alert_state)
        {
        case ALERT_NORMAL:
            __HAL_TIM_SET_AUTORELOAD(_tim, NORMAL_RATE);
            break;
        case ALERT_LOW:
            __HAL_TIM_SET_AUTORELOAD(_tim, LOW_ALERT_RATE);
            break;
        case ALERT_HIGH:
            __HAL_TIM_SET_AUTORELOAD(_tim, HIGH_ALERT_RATE);
            break;
        default:
            __HAL_TIM_SET_AUTORELOAD(_tim, NORMAL_RATE);
            break;
        }
    }
}

void alert_init(TIM_HandleTypeDef* timer_handle)
{
    _tim = timer_handle;
    change_state(ALERT_HIGH);
}

/**
 * @brief Checks if a value is consideret a hazard
 *
 * @param value float value of measured data
 * @param type type of measurement
 */
void alert_check(float value, alert_type_t type)
{
    switch (type)
    {
    case ALERT_TEMPERATURE:
        if (value < TEMPERATURE_LOW_ALERT) {
            change_state(ALERT_NORMAL);
        } else if ((value > TEMPERATURE_LOW_ALERT) && (value < TEMPERATURE_HIGH_ALERT)) {
            change_state(ALERT_LOW);
        } else if (value > TEMPERATURE_HIGH_ALERT) {
            change_state(ALERT_HIGH);
        }
        break;

    default:
        break;
    }
}
