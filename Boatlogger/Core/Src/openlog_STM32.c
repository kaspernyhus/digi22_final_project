/*
 * openlog_STM32.c
 *
 *  Created on: Nov 28, 2022
 *      Author: lpjensen
 */
#include "openlog_STM32.h"
#include <stdio.h>

extern UART_HandleTypeDef huart3;

/**
 * OpenLog Command mode
 */
void openlogCmdMode()
{
	uint8_t cmdBuf[3] = {0x1A, 0x1A, 0x1A};

    HAL_UART_Transmit(&huart3, (uint8_t*)cmdBuf, 3, 1000);
    HAL_Delay(30);
}

/**
 * Write a null terminated string to OpenLog
 */
void openlogWriteStr(char* string)
{
	uint8_t crBuf[1] = {0x0D};
	char* p = string;
	while(*p != '\0') {
		HAL_UART_Transmit(&huart3, (uint8_t*)p, 1, 1000);
		p++;
		HAL_Delay(5);
	}
	HAL_UART_Transmit(&huart3, (uint8_t*)crBuf, 1, 1000);
	HAL_Delay(5);
}


void openlogAppendFile(char* filename, char*string)
{
	char fileName[20];
	HAL_Delay(5);
	openlogCmdMode();
	sprintf(fileName, "new %s", filename);
	openlogWriteStr(fileName);
	HAL_Delay(10);
	sprintf(fileName, "append %s", filename);
	openlogWriteStr(fileName);
	HAL_Delay(5);
	openlogWriteStr(string);
	openlogCmdMode();
}
