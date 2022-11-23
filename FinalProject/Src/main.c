#include "stm32f30x_conf.h" // STM32 config
#include "30010_io.h"		// Input/output library for this course
#include "string.h"
#include "stdio.h"
#include "systick.h"
#include "sensor_module.h"
#include "bme280.h"
#include "i2c.h"

#define DISPLAY_MODULE_UPDATE_INTERVAL 10

uint16_t sensor_module_ticks = 0;
uint16_t openlog_module_run = 0;
uint16_t processing_module_run = 0;
uint16_t display_module_ticks = 0;

float new_data[MAX_NUMBER_OF_SENSORS];

/**
 * @brief Initializes the system
 *
 */
void system_init(void)
{
	systick_init();
	uart_init(9600);
	i2c_init();
}


// Move out of main.c
void register_sensors(void)
{
	// sensor_t temp_sensor = {
	// 	.name = "BME280 Temperature Sensor",
	// 	.init = bme280_temp_init,
	// 	.read = bme280_temp_read,
	// 	.is_available = bme280_temp_available
	// };
	// sensor_module_register(temp_sensor);
	// sensor_t hum_sensor = {
	// 	.name = "BME280 Humidity Sensor",
	// 	.init = bme280_hum_init,
	// 	.read = bme280_hum_read,
	// 	.is_available = bme280_hum_available
	// };
	// sensor_module_register(hum_sensor);
	// sensor_module_init();
}


int main(void)
{
	system_init();
	printf("---- Starting program ----\n");

	bme280_init();
	// register_sensors();

	while (1)
	{
		if (systick) {
			systick = 0;
			// printf("systick\n");
			// uint8_t data[2] = {BME280_CONTROL_REG, 0x00};
			// i2c_write(BME280_DEFAULT_ADDRESS, data, 2);

			// Sensor MODULE
			if (sensor_module_ticks >= SENSOR_MODULE_INTERVAL) {
				sensor_module_ticks = 0;
				// printf("Perform sensor reading\n");
				// sensor_module_read(new_data);
				float temp, pres, hum;
				bme280_burst_read(&temp, &pres, &hum);
				printf("Temperature: %.2f DegC\nPressure: %.2f hPa\nHumidity: %.2f %%RH", temp, pres, hum);
			}
			sensor_module_ticks++;

			// // OpenLog MODULE
			// if (openlog_module_run) {
			// 	openlog_module_run = 0;
			// 	printf("Saving data to SD card");
			// }

			// // Processing MODULE
			// if (processing_module_run) {
			// 	processing_module_run = 0;
			// 	printf("Processing sensor data");
			// }

			// // Display MODULE
			// if (display_module_ticks >= DISPLAY_MODULE_UPDATE_INTERVAL) {
			// 	display_module_ticks = 0;
			// 	printf("Updating display");
			// }
		}
	}
}
