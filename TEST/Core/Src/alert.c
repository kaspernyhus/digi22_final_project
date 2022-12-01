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
#include "stm32f3xx_hal.h"
#include "log.h"

static int tick_count = 0;
static uint8_t led_state = 0;
static uint8_t alert_tick_num;
alert_state_t current_alert_state = ALERT_HIGH;

static void change_state(alert_state_t new_state)
{
    if (new_state != current_alert_state) {
        current_alert_state = new_state;
        char buf[35];
        sprintf(buf, "New alert state: %s", alert_state_name[current_alert_state]);
        LOG(buf);
    }
}

void alert_init(void)
{
    change_state(ALERT_NORMAL);
}

void alert_tick(void)
{
    switch (current_alert_state)
    {
    case ALERT_NORMAL:
        alert_tick_num = NORMAL_TICKS;
        break;
    case ALERT_LOW:
        alert_tick_num = LOW_ALERT_TICKS;
        break;
    case ALERT_HIGH:
        alert_tick_num = HIGH_ALERT_TICKS;
        break;

    default:
        break;
    }

    if (tick_count > alert_tick_num) {
        tick_count = 0;
        if (led_state) {
            led_state = 0;
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, RESET);
        }
        else {
            led_state = 1;
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, SET);
        }
    }

    tick_count++;
}

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
