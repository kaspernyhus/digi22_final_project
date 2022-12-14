/*
 * bme280.h
 *
 *  Created on: 14. nov. 2022
 *      Author: kaspernyhus
 *
 * -------------
 *  I2C wiring
 * -------------
 *  Vin: 3-5V
 *  Gnd: Gnd
 *  SCK: SCL
 *  SDI: SDA
 * -------------
 * https://learn.adafruit.com/adafruit-bme280-humidity-barometric-pressure-temperature-sensor-breakout?view=all
 */

#ifndef BME280_H_
#define BME280_H_

#include <stdint.h>
#include "stm32f3xx_hal.h"
#include <stdbool.h>

#define BME280_ID 0x60

#define BME280_REGISTER_DIG_T1 0x88
#define BME280_REGISTER_DIG_T2 0x8A
#define BME280_REGISTER_DIG_T3 0x8C
#define BME280_REGISTER_DIG_P1 0x8E
#define BME280_REGISTER_DIG_P2 0x90
#define BME280_REGISTER_DIG_P3 0x92
#define BME280_REGISTER_DIG_P4 0x94
#define BME280_REGISTER_DIG_P5 0x96
#define BME280_REGISTER_DIG_P6 0x98
#define BME280_REGISTER_DIG_P7 0x9A
#define BME280_REGISTER_DIG_P8 0x9C
#define BME280_REGISTER_DIG_P9 0x9E
#define BME280_REGISTER_DIG_H1 0xA1
#define BME280_REGISTER_DIG_H2 0xE1
#define BME280_REGISTER_DIG_H3 0xE3
#define BME280_REGISTER_DIG_H4 0xE4
#define BME280_REGISTER_DIG_H5 0xE5
#define BME280_REGISTER_DIG_H6 0xE7

#define BME280_ALTERNATIVE_ADDRESS (0x76 << 1)
#define BME280_DEFAULT_ADDRESS (0x77 << 1)
#define BME280_CHIPID_REG 0xD0
#define BME280_CONTROL_HUM_REG 0xF2
#define BME280_STATUS_REG 0xF3
#define BME280_CONTROL_REG 0xF4
#define BME280_CONFIG_REG 0xF5
#define BME280_PRESS_DATA_REG 0xF7
#define BME280_TEMP_DATA_REG 0xFA
#define BME280_HUM_DATA_REG 0xFD

typedef struct {
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;
} bme280_calibration_data_t;

typedef enum {
    MODE_SLEEP,
    MODE_FORCED,
    MODE_NORMAL = 3
} bme280_mode_t;

typedef struct {
	float temperature;
	float pressure;
	float humidity;
} bme280_data_t;

typedef struct {
	I2C_HandleTypeDef i2c;
} bme280_t;

bool bme280_init(I2C_HandleTypeDef* i2c_handle);
uint8_t bme280_whoami(void);
void bme280_read_all(bme280_data_t* sensor_data);


#endif /* BME280_H_ */
