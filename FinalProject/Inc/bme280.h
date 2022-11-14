/*
 * bme280.h
 *
 *  Created on: 14. nov. 2022
 *      Author: kaspernyhus
 */

#ifndef BME280_H_
#define BME280_H_

#include <stdint.h>

void bme280_temp_init(void);
float bme280_temp_read(void);
uint8_t bme280_temp_available(void);
void bme280_hum_init(void);
float bme280_hum_read(void);
uint8_t bme280_hum_available(void);

#endif /* BME280_H_ */
