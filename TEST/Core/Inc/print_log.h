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
#include <stdint.h>
#include "stm32f3xx_hal.h"

typedef struct {
    int hours;
	int min;
	int sec;
} log_time_t;

void print_log_init(UART_HandleTypeDef* uart_handle, log_time_t* time, uint32_t* systicks);

/**
 * @brief Logs a string to the terminal
 *
 * @param str
 */
void printInfo(char* str);

/**
 * @brief Logs a string to the terminal
 *
 * @param str
 */
void printError(char* str);
