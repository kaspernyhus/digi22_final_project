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

#define MAX_ALERTS 20

// LED parameters
#define NORMAL_RATE 30000-1
#define LOW_ALERT_RATE 2000-1
#define HIGH_ALERT_RATE 600-1

typedef enum {
    ALERT_NORMAL,
    ALERT_LOW,
    ALERT_HIGH
} alert_state_t;

static const char* const alert_state_name[3] = {"ALERT_NORMAL", "ALERT_LOW", "ALERT_HIGH"};

typedef enum {
    TEMPERATURE_ALERT,
    HUMIDITY_ALERT,
    PRESSURE_ALERT,
    WATER_LEVEL_ALERT,
    BATTERY_VOLTAGE_ALERT,
    POSITION_ALERT,
    SPEED_ALERT
} alert_type_t;

typedef struct {
    char* name;
    alert_type_t alert_type;
    float low_thresshold;
    float high_thresshold;
    alert_state_t alert_state;
} alert_t;

void alert_system_init(TIM_HandleTypeDef* timer_handle);
void alert_system_register(alert_type_t alert_type, char* name, float low_thresshold, float high_thresshold);
void alert_system_check(float value, alert_type_t type);
