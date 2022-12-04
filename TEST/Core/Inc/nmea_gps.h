/*
 * nmea_gps.h
 *
 *  Created on: 29. nov. 2022
 *      Author: lpjensen
 *
 *      Parts of the code is taken from u-blox.com opensource GPS roll-over fix
 *      "GPS week number roll-over workaround - Application Note"
 */

#ifndef INC_NMEA_GPS_H_
#define INC_NMEA_GPS_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stm32f3xx_hal.h"

typedef struct {
	int hours;
	int min;
	int sec;
} gps_time_t;

typedef struct{
	float lati;
	float longi;
	float speed;
	float course;
	unsigned int date;
	gps_time_t time;
} gps_data_t;

void formatData(float time, gps_data_t* gps_data);
void getLocation(gps_data_t* gps_data, char* rxData);
uint16_t day_number_1980(uint16_t yyyy, uint16_t mm, uint16_t dd);
void date_1980(uint16_t days, uint16_t *year, uint16_t *month, uint16_t *day);
void gprmc2int(char gprmc[], uint16_t *year, uint16_t *month, uint16_t *day);
char *int2gprmc(uint16_t year, uint16_t month, uint16_t day);
unsigned int rolloverDateConvertion(unsigned int date);

#endif /* INC_NMEA_GPS_H_ */
