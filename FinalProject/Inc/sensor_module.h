/*
 * sensor_module.h
 *
 *  Created on: 14. nov. 2022
 *      Author: kaspernyhus
 */

#ifndef SENSOR_MODULE_H_
#define SENSOR_MODULE_H_

#include <stdint.h>
#include "system.h"

#define MAX_NUMBER_OF_SENSORS 3
#define SENSOR_MODULE_INTERVAL 3

typedef float (*sensor_read)(void);
typedef void (*sensor_init)(void);
typedef uint8_t (*sensor_available)(void);

typedef struct {
	char* name;
	sensor_init init;
	sensor_read read;
	sensor_available is_available;
} sensor_t;

error_t sensor_module_register(sensor_t sensor);
void sensor_module_init(void);
void sensor_module_read(float* new_sensor_data);

#endif /* SENSOR_MODULE_H_ */
