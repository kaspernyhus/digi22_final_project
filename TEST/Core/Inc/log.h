/**
 * @file log.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-12-01
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include <stdio.h>
#include "stm32f3xx_hal.h"


void LOG_init(UART_HandleTypeDef* uart_handle);

/**
 * @brief Logs a string to the terminal
 *
 * @param str
 */
void LOG(char* str);
