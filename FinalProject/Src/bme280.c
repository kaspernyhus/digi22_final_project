/*
 * bme280.c
 *
 *  Created on: 14. nov. 2022
 *      Author: kaspernyhus
 */

#include "bme280.h"
#include <stdio.h>
#include "i2c.h"

bme280_calibration_data_t bme280_calibration_data;

void bme280_read_calibration_data(void)
{
	uint8_t calib_data[33];
	i2c_read(BME280_DEFAULT_ADDRESS, BME280_REGISTER_DIG_T1, calib_data, 33);

	bme280_calibration_data.dig_T1 = (calib_data[1]<<8) | calib_data[0];
	bme280_calibration_data.dig_T2 = (calib_data[3]<<8) | calib_data[2];
	bme280_calibration_data.dig_T3 = (calib_data[5]<<8) | calib_data[4];
	bme280_calibration_data.dig_P1 = (calib_data[7]<<8) | calib_data[6];
	bme280_calibration_data.dig_P2 = (calib_data[9]<<8) | calib_data[8];
	bme280_calibration_data.dig_P3 = (calib_data[11]<<8) | calib_data[10];
	bme280_calibration_data.dig_P4 = (calib_data[13]<<8) | calib_data[12];
	bme280_calibration_data.dig_P5 = (calib_data[15]<<8) | calib_data[14];
	bme280_calibration_data.dig_P6 = (calib_data[17]<<8) | calib_data[16];
	bme280_calibration_data.dig_P7 = (calib_data[19]<<8) | calib_data[18];
	bme280_calibration_data.dig_P8 = (calib_data[21]<<8) | calib_data[20];
	bme280_calibration_data.dig_P9 = (calib_data[23]<<8) | calib_data[22];
	bme280_calibration_data.dig_H1 = calib_data[24];
	bme280_calibration_data.dig_H2 = (calib_data[26]<<8) | calib_data[25];
	bme280_calibration_data.dig_H3 = calib_data[27];
	bme280_calibration_data.dig_H4 = (calib_data[29]<<8) | calib_data[28];
	bme280_calibration_data.dig_H5 = (calib_data[31]<<8) | calib_data[30];
	bme280_calibration_data.dig_H6 = calib_data[32];
	printf("BME280 calibration data read\n");
}

void bme280_force_measurement(void)
{
	uint8_t crtl_meas;
	uint8_t data[2];
	// Read register on BME280
	i2c_read(BME280_DEFAULT_ADDRESS, BME280_CONTROL_REG, &crtl_meas, 1);
	// Rewrite forced
	data[0] = BME280_CONTROL_REG;
	crtl_meas |= MODE_FORCED;
	data[1] = crtl_meas;
	i2c_write(BME280_DEFAULT_ADDRESS, data, 2);
}

void bme280_init(void)
{
	printf("Initializing temperature sensor\n");
	uint8_t chip_id = 0;
	uint8_t crtl_hum = 0x00;
	uint8_t crtl_meas = 0x00;
	uint8_t data[4];

	// Check chip id
	i2c_read(BME280_DEFAULT_ADDRESS, BME280_CHIPID_REG, &chip_id, 1);
	printf("BME280_CHIPID_REG: 0x%.2X\n", chip_id);

	if(chip_id != 0x60) {
		printf("Chip not available\n");
		return;
	}

	// set BME280_CONTROL_HUM_REG 0xF2
	data[0] = BME280_CONTROL_HUM_REG;
	// Humidity oversampling x 1
	crtl_hum |= 0b00000001;
	data[1] = crtl_hum;

	// set BME280_CONTROL_REG 0xF4
	data[2] = BME280_CONTROL_REG;
	crtl_meas = 0x00;
	// Mode FORCED
	crtl_meas |= MODE_FORCED;
	// Pressure oversampling x 1
	crtl_meas |= 0b00000100;
	// Temperature oversampling x 1
	crtl_meas |= 0b00100000;
	data[3] = crtl_meas;
	i2c_write(BME280_DEFAULT_ADDRESS, data, 4);

	// Read chips NVM calibration data
	bme280_read_calibration_data();
}

void bme280_burst_read(void)
{
	bme280_force_measurement();
	uint8_t status = 0;
	uint8_t rcv_buffer[8] = {0};

	i2c_read(BME280_DEFAULT_ADDRESS, BME280_STATUS_REG, &status, 1);

	while (status == 0)
	{
		i2c_read(BME280_DEFAULT_ADDRESS, BME280_STATUS_REG, &status, 1);
	}

	// read 8 bytes starting from BME280_PRESS_DATA_REG
	i2c_read(BME280_DEFAULT_ADDRESS, BME280_PRESS_DATA_REG, rcv_buffer, 8);

	for(int i=0; i<8; i++) {
		printf("%.2X ", rcv_buffer[i]);
	}
	printf("\n");

	// Pressure
	uint32_t raw_pressure = ((rcv_buffer[0]<<16) | (rcv_buffer[1]<<8) | (rcv_buffer[2]))>>4;
	printf("Pressure: 0x%08lX\n", raw_pressure);

	// Temperature
	int32_t var1, var2;
	u_int32_t adc_T = ((rcv_buffer[3]<<16) | (rcv_buffer[4]<<8) | (rcv_buffer[5]))>>4;
	printf("Temperature: 0x%08lX\n", adc_T);
	// Calculations from DS p.25
	var1 = ((((adc_T>>3) - ((int32_t)bme280_calibration_data.dig_T1<<1))) * ((int32_t)bme280_calibration_data.dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)bme280_calibration_data.dig_T1)) * ((adc_T>>4) - ((int32_t)bme280_calibration_data.dig_T1))) >> 12) * ((int32_t)bme280_calibration_data.dig_T3)) >> 14;
	int32_t t_fine = var1 + var2;
	int32_t T = (t_fine * 5 + 128) >> 8;
	float temperature = (float)T / 100;
	printf("Temperature: %.2f\n", temperature);

	// Humidity
	uint16_t raw_humidity = (rcv_buffer[6]<<8) | (rcv_buffer[7]);
	printf("Humidity: 0x%04x\n", raw_humidity);
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
