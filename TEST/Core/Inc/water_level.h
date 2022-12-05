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

#define WATER_LEVEL_THRESHOLD 600

typedef enum {
    WATER_LEVEL_LOW,
    WATER_LEVEL_HIGH
} water_level_t;

static const char* const waterlevel_str[2] = {"Low", "High"};

water_level_t water_level_convert(uint16_t adc_reading);
