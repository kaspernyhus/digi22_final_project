/**
 * @file i2c-lcd.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-30
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once
#include "stm32f3xx_hal.h"

#define SLAVE_ADDRESS_LCD (0x3F << 1)

typedef enum {
    DONT_CLEAR_LCD,
    CLEAR_LCD
} lcd_clear_t;

void lcd_init(I2C_HandleTypeDef* i2c_handle);
void lcd_send_string_xy(char *str, int row, int col, lcd_clear_t clear);
void lcd_clear(void);
