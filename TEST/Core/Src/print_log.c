/**
 * @file log.c
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-12-01
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "print_log.h"
#include <string.h>

static UART_HandleTypeDef* _uart;
static log_time_t* _time;
static uint32_t* _systicks_cnt;

/**
 * @brief Initialize logger
 *
 * @param uart_handle UART_HandleTypeDef to uart peripheral
 * @param time Pointer to time struct (hour:min:sec)
 */
void print_log_init(UART_HandleTypeDef* uart_handle, log_time_t* time, uint32_t* systicks_cnt)
{
    _uart = uart_handle;
    _time = time;
    _systicks_cnt = systicks_cnt;
    printError("Test");
}

/**
 * @brief Log a string to the terminal, INFO level
 *
 * @param str
 */
void printInfo(char* str)
{
  char pre_log[40];

  // sprintf(time, "[%lu] (%.2d:%.2d:%.2d): ", *_systicks_cnt, _time->hours, _time->min, _time->sec);
  // HAL_UART_Transmit(_uart, (uint8_t*)time, 22, 100);
  sprintf(pre_log, "I [%lu] (%.2d:%.2d:%.2d): ", *_systicks_cnt, _time->hours, _time->min, _time->sec);
  HAL_UART_Transmit(_uart, (uint8_t*)pre_log, strlen(pre_log), 100);
  HAL_UART_Transmit(_uart, (uint8_t*)str, strlen(str), 100);
  HAL_UART_Transmit(_uart, (uint8_t*)"\r\n", 2, 100);
}

/**
 * @brief Log a string to the terminal, ERROR level
 *
 * @param str
 */
void printError(char* str)
{
  char pre_log[40];

  // sprintf(time, "[%lu] (%.2d:%.2d:%.2d): ", *_systicks_cnt, _time->hours, _time->min, _time->sec);
  // HAL_UART_Transmit(_uart, (uint8_t*)time, 22, 100);
  sprintf(pre_log, "E [%lu] (%.2d:%.2d:%.2d): ", *_systicks_cnt, _time->hours, _time->min, _time->sec);
  HAL_UART_Transmit(_uart, (uint8_t*)pre_log, strlen(pre_log), 100);
  HAL_UART_Transmit(_uart, (uint8_t*)str, strlen(str), 100);
  HAL_UART_Transmit(_uart, (uint8_t*)"\r\n", 2, 100);
}
