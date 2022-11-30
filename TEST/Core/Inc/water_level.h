/**
 * @file water_level.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-30
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include <stdint.h>

typedef enum {
    WATER_LEVEL_LOW,
    WATER_LEVEL_HIGH
} water_level_t;

static const char* const waterlevel_str[2] = {"Low", "High"};

water_level_t check_water_level(uint16_t adc_reading);
