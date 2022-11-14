#include "stm32f30x_conf.h" // STM32 config
#include "30010_io.h"		// Input/output library for this course
#include "string.h"
#include "stdio.h"
#include "systick.h"
#include "sensor_module.h"
#include "bme280.h"

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

	// Sensor MODULE
	sensor_t temp_sensor = {
		.init = bme280_temp_init,
		.read = bme280_temp_read,
		.is_available = bme280_temp_available
	};
	sensor_module_register(temp_sensor);
	sensor_module_init();
}

int main(void)
{
	system_init();
	printf("---- Starting program ----\n");

	while (1)
	{
		if (systick) {
			systick = 0;
			// printf("systick\n");

			// Sensor MODULE
			if (sensor_module_ticks >= SENSOR_MODULE_INTERVAL) {
				sensor_module_ticks = 0;
				printf("Perform sensor reading\n");
				sensor_module_read(new_data);
			}
			sensor_module_ticks++;

			// OpenLog MODULE
			if (openlog_module_run) {
				openlog_module_run = 0;
				printf("Saving data to SD card");
			}

			// Processing MODULE
			if (processing_module_run) {
				processing_module_run = 0;
				printf("Processing sensor data");
			}

			// Display MODULE
			if (display_module_ticks >= DISPLAY_MODULE_UPDATE_INTERVAL) {
				display_module_ticks = 0;
				printf("Updating display");
			}
		}
	}
}
