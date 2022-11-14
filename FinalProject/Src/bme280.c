/*
 * bme280.c
 *
 *  Created on: 14. nov. 2022
 *      Author: kaspernyhus
 */

#include "bme280.h"
#include <stdio.h>

void bme280_temp_init(void)
{
	printf("Initializing temperature sensor\n");
}

float bme280_temp_read(void)
{
	float temperature = 23.234234;
	return temperature;
}

uint8_t bme280_temp_available(void)
{
	return 1;
}


void bme280_hum_init(void)
{
	printf("Initializing humidity sensor\n");
}

float bme280_hum_read(void)
{
	float humidity = 89.4234;
	return humidity;
}

uint8_t bme280_hum_available(void)
{
	return 1;
}
