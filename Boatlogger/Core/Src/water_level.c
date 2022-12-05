/**
 * @file water_level.c
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-30
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "water_level.h"

water_level_t water_level_convert(uint16_t adc_reading)
{
    if (adc_reading <= WATER_LEVEL_THRESHOLD) {
        return WATER_LEVEL_LOW;
    } else {
        return WATER_LEVEL_HIGH;
    }
}
