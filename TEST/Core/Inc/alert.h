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

#define NORMAL_TICKS 4
#define LOW_ALERT_TICKS 2
#define HIGH_ALERT_TICKS 1

#define TEMPERATURE_LOW_ALERT 25.0
#define TEMPERATURE_HIGH_ALERT 28.0

typedef enum {
  ALERT_NORMAL,
  ALERT_LOW,
  ALERT_HIGH
} alert_state_t;

typedef enum {
    ALERT_TEMPERATURE,

} alert_type_t;

void alert_init(void);
void alert_tick(void);
void alert_check(float value, alert_type_t type);

