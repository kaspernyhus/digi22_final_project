/*
 * sensor_module.c
 *
 *  Created on: 14. nov. 2022
 *      Author: kaspernyhus
 */

#include "sensor_module.h"
#include <string.h>
#include <stdio.h>

uint8_t sensors_registered = 0;
sensor_t sensors[MAX_NUMBER_OF_SENSORS];
float sensor_data[MAX_NUMBER_OF_SENSORS];

/**
 * @brief Add sensor to list of sensors
 *
 * @param sensor to be added
 * @return error_t
 */
error_t sensor_module_register(sensor_t sensor)
{
	if (sensors_registered < MAX_NUMBER_OF_SENSORS) {
		sensors[sensors_registered] = sensor;
	}
	sensors_registered++;
	return SYS_OK;
}

/**
 * @brief Calls init function of sensor
 *
 */
void sensor_module_init(void)
{
    for (int i=0; i<sensors_registered; i++) {
		sensors[i].init();
	}
}

/**
 * @brief Reads data from the collection of sensors
 *
 * @param new_sensor_data outputs all new sensor data
 */
void sensor_module_read(float* new_sensor_data)
{
	for (int i=0; i<sensors_registered; i++) {
        if (sensors[i].is_available()) {
		    sensor_data[i] = sensors[i].read();
		    printf("%s reading = %.2f\n", sensors[i].name, sensor_data[i]);
        }
	}
    memcpy(new_sensor_data, sensor_data, sizeof(sensor_data));
}

