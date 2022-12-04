/**
 * @file alert.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-12-01
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include "stm32f3xx_hal.h"

#define NORMAL_RATE 30000-1
#define LOW_ALERT_RATE 2000-1
#define HIGH_ALERT_RATE 600-1

#define TEMPERATURE_LOW_ALERT 23.0
#define TEMPERATURE_HIGH_ALERT 27.0

typedef enum {
  ALERT_NORMAL,
  ALERT_LOW,
  ALERT_HIGH
} alert_state_t;

static const char* const alert_state_name[3] = {"ALERT_NORMAL", "ALERT_LOW", "ALERT_HIGH"};

typedef enum {
    ALERT_TEMPERATURE,

} alert_type_t;

void alert_init(TIM_HandleTypeDef* timer_handle);
void alert_check(float value, alert_type_t type);
