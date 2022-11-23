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
int32_t t_fine = 0;

void bme280_read_calibration_data(void)
{
	uint8_t calib_data[2];
	i2c_read(BME280_DEFAULT_ADDRESS, BME280_REGISTER_DIG_T1, calib_data, 2);
	bme280_calibration_data.dig_T1 = (calib_data[1]<<8) | calib_data[0];

	i2c_read(BME280_DEFAULT_ADDRESS, BME280_REGISTER_DIG_T2, calib_data, 2);
	bme280_calibration_data.dig_T2 = (calib_data[1]<<8) | calib_data[0];

	i2c_read(BME280_DEFAULT_ADDRESS, BME280_REGISTER_DIG_T3, calib_data, 2);
	bme280_calibration_data.dig_T3 = (calib_data[1]<<8) | calib_data[0];

	i2c_read(BME280_DEFAULT_ADDRESS, BME280_REGISTER_DIG_P1, calib_data, 2);
	bme280_calibration_data.dig_P1 = (calib_data[1]<<8) | calib_data[0];

	i2c_read(BME280_DEFAULT_ADDRESS, BME280_REGISTER_DIG_P2, calib_data, 2);
	bme280_calibration_data.dig_P2 = (calib_data[1]<<8) | calib_data[0];

	i2c_read(BME280_DEFAULT_ADDRESS, BME280_REGISTER_DIG_P3, calib_data, 2);
	bme280_calibration_data.dig_P3 = (calib_data[1]<<8) | calib_data[0];

	i2c_read(BME280_DEFAULT_ADDRESS, BME280_REGISTER_DIG_P4, calib_data, 2);
	bme280_calibration_data.dig_P4 = (calib_data[1]<<8) | calib_data[0];

	i2c_read(BME280_DEFAULT_ADDRESS, BME280_REGISTER_DIG_P5, calib_data, 2);
	bme280_calibration_data.dig_P5 = (calib_data[1]<<8) | calib_data[0];

	i2c_read(BME280_DEFAULT_ADDRESS, BME280_REGISTER_DIG_P6, calib_data, 2);
	bme280_calibration_data.dig_P6 = (calib_data[1]<<8) | calib_data[0];

	i2c_read(BME280_DEFAULT_ADDRESS, BME280_REGISTER_DIG_P7, calib_data, 2);
	bme280_calibration_data.dig_P7 = (calib_data[1]<<8) | calib_data[0];

	i2c_read(BME280_DEFAULT_ADDRESS, BME280_REGISTER_DIG_P8, calib_data, 2);
	bme280_calibration_data.dig_P8 = (calib_data[1]<<8) | calib_data[0];

	i2c_read(BME280_DEFAULT_ADDRESS, BME280_REGISTER_DIG_P9, calib_data, 2);
	bme280_calibration_data.dig_P9 = (calib_data[1]<<8) | calib_data[0];

	i2c_read(BME280_DEFAULT_ADDRESS, BME280_REGISTER_DIG_H1, calib_data, 1);
	bme280_calibration_data.dig_H1 = calib_data[0];

	i2c_read(BME280_DEFAULT_ADDRESS, BME280_REGISTER_DIG_H2, calib_data, 2);
	bme280_calibration_data.dig_H2 = (calib_data[1]<<8) | calib_data[0];

	i2c_read(BME280_DEFAULT_ADDRESS, BME280_REGISTER_DIG_H3, calib_data, 1);
	bme280_calibration_data.dig_H3 = calib_data[0];

	i2c_read(BME280_DEFAULT_ADDRESS, BME280_REGISTER_DIG_H4, calib_data, 2);
	bme280_calibration_data.dig_H4 = ((int8_t)calib_data[0]<<4) | (calib_data[1]&0xF);

	i2c_read(BME280_DEFAULT_ADDRESS, BME280_REGISTER_DIG_H5, calib_data, 2);
	bme280_calibration_data.dig_H5 = ((int8_t)calib_data[1]<<4) | (calib_data[0]>>4);

	i2c_read(BME280_DEFAULT_ADDRESS, BME280_REGISTER_DIG_H6, calib_data, 1);
	bme280_calibration_data.dig_H6 = calib_data[0];

	printf("BME280 calibration data read\n");
}

void bme280_force_measurement(void)
{
	uint8_t crtl_hum;
	uint8_t crtl_meas;
	uint8_t data[4];

	// set BME280_CONTROL_HUM_REG 0xF2
	data[0] = BME280_CONTROL_HUM_REG;
	// Humidity oversampling x 1
	crtl_hum = 0b00000001;
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
}

void bme280_init(void)
{
	printf("Initializing temperature sensor\n");
	uint8_t chip_id = 0;

	// Check chip id
	i2c_read(BME280_DEFAULT_ADDRESS, BME280_CHIPID_REG, &chip_id, 1);
	printf("BME280_CHIPID_REG: 0x%.2X - %s\n", chip_id, chip_id==0x60 ? "OK" : "Error");

	if(chip_id != 0x60) {
		printf("Chip not available\n");
		return;
	}

	bme280_force_measurement();

	// Read chips NVM calibration data
	bme280_read_calibration_data();
}

// Temperature in DegC, resolution 0.01 DegC.
float calculate_temperatur(u_int32_t adc_T)
{
	int32_t var1, var2;

	// Calculations from DS p.25
	var1 = ((((adc_T>>3) - ((int32_t)bme280_calibration_data.dig_T1<<1))) * ((int32_t)bme280_calibration_data.dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)bme280_calibration_data.dig_T1)) * ((adc_T>>4) - ((int32_t)bme280_calibration_data.dig_T1))) >> 12) * ((int32_t)bme280_calibration_data.dig_T3)) >> 14;
	t_fine = var1 + var2;
	int32_t T = (t_fine * 5 + 128) >> 8;
	return (float)T / 100;
}

// Pressure in hPa
float calculate_pressure(uint32_t adc_P)
{
	float pressure = 0;
	int64_t var1, var2;

	// Calculations from DS p.25
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)bme280_calibration_data.dig_P6;
	var2 = var2 + ((var1*(int64_t)bme280_calibration_data.dig_P5)<<17);
	var2 = var2 + (((int64_t)bme280_calibration_data.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)bme280_calibration_data.dig_P3)>>8) + ((var1 * (int64_t)bme280_calibration_data.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bme280_calibration_data.dig_P1)>>33;
	if (var1 != 0) { // avoid exception caused by division by zero
		int64_t p = 1048576-adc_P;
		p = (((p<<31)-var2)*3125)/var1;
		var1 = (((int64_t)bme280_calibration_data.dig_P9) * (p>>13) * (p>>13)) >> 25;
		var2 = (((int64_t)bme280_calibration_data.dig_P8) * p) >> 19;
		p = ((p + var1 + var2) >> 8) + (((int64_t)bme280_calibration_data.dig_P7)<<4);
		pressure = p / 256.0 / 1000;
	}
	return pressure;
}

// Humidity in %RH
float calculate_humidity(int32_t adc_H)
{
	int32_t v_x1_u32r;
	// Calculations from DS p.25-26
	v_x1_u32r = (t_fine - ((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)bme280_calibration_data.dig_H4) << 20) - (((int32_t)bme280_calibration_data.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)bme280_calibration_data.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)bme280_calibration_data.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)bme280_calibration_data.dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)bme280_calibration_data.dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	uint32_t H = (uint32_t)(v_x1_u32r>>12);
	return (float)H / 1024.0;
}

void bme280_burst_read(float* temperature, float* pressure, float* humidity)
{
	bme280_force_measurement();
	uint8_t status = 0;
	uint8_t rcv_buffer[8] = {0};

	i2c_read(BME280_DEFAULT_ADDRESS, BME280_STATUS_REG, &status, 1);

	// Wait till sensor sets status register bit == measurement ready
	while (status == 0)
	{
		i2c_read(BME280_DEFAULT_ADDRESS, BME280_STATUS_REG, &status, 1);
	}

	// read 8 bytes starting from BME280_PRESS_DATA_REG
	i2c_read(BME280_DEFAULT_ADDRESS, BME280_PRESS_DATA_REG, rcv_buffer, 8);

	// Print data bytes
	// for(int i=0; i<8; i++) {
	// 	printf("%.2X ", rcv_buffer[i]);
	// }
	// printf("\n");

	u_int32_t adc_T = ((rcv_buffer[3]<<16) | (rcv_buffer[4]<<8) | (rcv_buffer[5]))>>4;
	float _temperature = calculate_temperatur(adc_T);
	*temperature = _temperature;
	// printf("Temperature: %.2f DegC\n", temperature);

	uint32_t adc_P = ((rcv_buffer[0]<<16) | (rcv_buffer[1]<<8) | (rcv_buffer[2]))>>4;
	float _pressure = calculate_pressure(adc_P);
	*pressure = _pressure;
	// printf("Pressure: %.2f hPa\n", pressure);

	int32_t adc_H = (rcv_buffer[6]<<8) | (rcv_buffer[7]);
	float _humidity = calculate_humidity(adc_H);
	*humidity = _humidity;
	// printf("Humidity: %.2f %%RH\n", humidity);
}
