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

#include "log.h"
#include <string.h>

static UART_HandleTypeDef* _uart;
static log_time_t* _time;

/**
 * @brief Initialize logger
 *
 * @param uart_handle UART_HandleTypeDef to uart peripheral
 * @param time Pointer to time struct (hour:min:sec)
 */
void LOG_init(UART_HandleTypeDef* uart_handle, log_time_t* time)
{
    _uart = uart_handle;
    _time = time;
}

/**
 * @brief Log a string to the terminal
 *
 * @param str
 */
void LOG(char* str)
{
  char time[19];
  sprintf(time, "(%.2d:%.2d:%.2d): ", _time->hours, _time->min, _time->sec);
  size_t str_length = strlen(str);
  HAL_UART_Transmit(_uart, (uint8_t*)time, 12, 100);
  HAL_UART_Transmit(_uart, (uint8_t*)str, str_length, 100);
  HAL_UART_Transmit(_uart, (uint8_t*)"\r\n", 2, 100);
}
