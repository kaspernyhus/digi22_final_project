/*
 * openlog_STM32.h
 *
 *  Created on: Nov 28, 2022
 *      Author: lpjensen
 */
#include "stm32f3xx_hal.h"

#ifndef INC_OPENLOG_STM32_H_
#define INC_OPENLOG_STM32_H_

void openlogCmdMode();									//Put openlog in commmand mode

void openlogWriteStr(char* string);						//Write string to openlog

void openlogAppendFile(char* filename, char*string); 	//Create and append data to a file

#endif /* INC_OPENLOG_STM32_H_ */
