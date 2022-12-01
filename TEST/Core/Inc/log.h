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

typedef struct {
    int hours;
	int min;
	int sec;
} log_time_t;

void LOG_init(UART_HandleTypeDef* uart_handle, log_time_t* time);

/**
 * @brief Logs a string to the terminal
 *
 * @param str
 */
void LOG(char* str);
