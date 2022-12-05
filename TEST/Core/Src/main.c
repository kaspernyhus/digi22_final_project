/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "print_log.h"
#include "i2c-lcd.h"
#include "openlog_STM32.h"
#include "bme280.h"
#include "nmea_gps.h"
#include "water_level.h"
#include "alert_system.h"
#include "button.h"
#include "gps_lock.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SYS_TICK_INTERVAL_MS 40
#define SENSOR_READ_RATE 5000/SYS_TICK_INTERVAL_MS
#define LOG_DATA_RATE 10000/SYS_TICK_INTERVAL_MS
#define GPSBUF_SIZE 500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

// Buffers
uint8_t rx_buf[GPSBUF_SIZE]; //Buffer to hold raw gps data
char gps_buf[GPSBUF_SIZE];	//Buffer for GPS data processing
char lcdBuf[20];
bme280_data_t bme280_data;
gps_data_t gps_data = {.time.hours = 00, .time.min = 00, .time.sec = 00}; // parsed gps data
uint16_t waterlevel_reading = 0;
water_level_t water_level = WATER_LEVEL_LOW;
uint16_t adcCh2 = 0;
float batVol = 0;

float lock_pos_lati = 0;
float lock_pos_long = 0;

// Systick
volatile uint8_t systick = 0;
uint32_t systick_cnt = 0;

// Flags
volatile uint8_t gps_sat_lock = 0;		//EXT interrupt when GPS has lock
volatile uint8_t gps_data_ready = 0;	//DMA interrupt when UART line goes idle
volatile uint8_t gps_active = 0;		//GPS data is processed and ready to log and display
static uint8_t read_sensor_cnt = SENSOR_READ_RATE;
static uint8_t log_data_cnt = 0;
static uint8_t check_water_lvl_cnt = 0;
static uint8_t lcd_update_page = 1;
static alert_state_t alert_state_global = ALERT_NORMAL;

// LCD control
typedef enum {
    LCD_PAGE_NO_GPS,
    LCD_PAGE_TIME, // Require GPS lock
    LCD_PAGE_GPS, // Require GPS lock
    LCD_PAGE_SPEED, // Require GPS lock
    LCD_PAGE_TEMP,
    LCD_PAGE_WATER_LVL,
    LCD_PAGE_BAT_VOL,
    LCD_PAGE_ALERTS
} lcd_page_t;

lcd_page_t lcd_current_page = LCD_PAGE_NO_GPS;

/**
 * @brief Go to next lcd page.
 *
 */
void lcd_toggle(void)
{
    lcd_page_t lcd_next_page;
    switch (lcd_current_page)
    {
    case LCD_PAGE_TEMP:
        if (alert_state_global) {
            lcd_next_page = LCD_PAGE_ALERTS;
        } else {
            lcd_next_page = LCD_PAGE_WATER_LVL;
        }
        break;
    case LCD_PAGE_WATER_LVL:
        lcd_next_page = LCD_PAGE_BAT_VOL;
        break;
    case LCD_PAGE_BAT_VOL:
        if (!gps_active) {
            lcd_next_page = LCD_PAGE_NO_GPS;
        } else {
            lcd_next_page = LCD_PAGE_TIME;
        }
        break;
    case LCD_PAGE_NO_GPS:
        lcd_next_page = LCD_PAGE_TEMP;
        break;
    case LCD_PAGE_TIME:
        if (!gps_active) {
            lcd_next_page = LCD_PAGE_TEMP;
        } else {
            lcd_next_page = LCD_PAGE_GPS;
        }
        break;
    case LCD_PAGE_GPS:
        if (!gps_active) {
            lcd_next_page = LCD_PAGE_TEMP;
        } else {
            lcd_next_page = LCD_PAGE_SPEED;
        }
        break;
    case LCD_PAGE_SPEED:
        lcd_next_page = LCD_PAGE_TEMP;
        break;
    case LCD_PAGE_ALERTS:
        lcd_next_page = LCD_PAGE_WATER_LVL;
        break;
    default:
        lcd_next_page = LCD_PAGE_TEMP;
        break;
    }
    // char buf[50];
    // sprintf(buf, "Current page: %d // Next page: %d", lcd_current_page, lcd_next_page);
    // printInfo(buf);
    lcd_current_page = lcd_next_page;
    lcd_update_page = 1;
}

// User button
button_t user_button;
button_pin_state_t read_user_button(void)
{
  return (button_pin_state_t)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
}

void button_pressed(void)
{
    lcd_toggle();
}

void button_long_pressed(void)
{
    // TODO: Save and lock GPS location
    printInfo("User Button long press!");
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    char logData[100];

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  MX_TIM15_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

    HAL_TIM_Base_Start_IT(&htim2);	//Starting Timer2 in interrupt mode
    HAL_TIM_Base_Start_IT(&htim15);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buf, GPSBUF_SIZE); 	//Init GPS uart data to DMA
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);				//Disable half-tranfer complete interrupt, since data size is unknown

    // Initialize terminal logger
    print_log_init(&huart2, (log_time_t*)&gps_data.time, &systick_cnt);
    printInfo("Boat-log ON!");

    // LCD
    lcd_init(&hi2c1);
    lcd_send_string_xy("Boat-log ON!", 0, 0, CLEAR_LCD);
    HAL_Delay(2000);
    lcd_clear();

    // User button
    button_config_t button_cfg = {
        .active_state = BUTTON_PIN_STATE_LOW,
        .read_pin_cb = &read_user_button,
        .debounce_ticks = 0,
        .long_press_ticks = 16
    };
    button_init(&user_button, &button_cfg);
    button_register_cb(&user_button, &button_pressed, BUTTON_ON_SHORT_PRESS);
    button_register_cb(&user_button, &button_long_pressed, BUTTON_ON_LONG_PRESS);

    // BME280 temperature/humudity/pressure sensor
    if (bme280_init(&hi2c1)) {
        printInfo("BME280 initialized");
    }

    // OpenLog
    //Header for .csv logfile
    sprintf(logData, "Time,Date,Latitude,Longitude,Speed,Course,Temp,Pres,Hum,WaterLvl,ADC_Ch2");
    openlogAppendFile("log1.csv", logData);
    printInfo("Started OpenLog file");

    // Alert system
    alert_system_init(&htim15);
    printInfo("Initialized alert system");

    // Set alerts
    alert_system_register(TEMPERATURE_ALERT, "Temperature", ALERT_ABOVE_THRESHOLD, 25.0, 28.0);
    alert_system_register(HUMIDITY_ALERT, "Humidity", ALERT_ABOVE_THRESHOLD, 72.0, 90.0);
    alert_system_register(BATTERY_VOLTAGE_ALERT, "Low Battery Voltage", ALERT_BELOW_THRESHOLD, 10.8, 11.2);
    alert_system_register(BATTERY_VOLTAGE_ALERT, "Overvoltage protection", ALERT_ABOVE_THRESHOLD, 12.5, 13.0);
    alert_system_register(WATER_LEVEL_ALERT, "Water level", ALERT_ABOVE_THRESHOLD, 1, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        if (systick) {
            systick = 0;
            button_tick(&user_button);
            systick_cnt++;
            read_sensor_cnt++;
            log_data_cnt++;
            check_water_lvl_cnt++;
            alert_state_global = alert_system_alert_level();
        }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        if(gps_sat_lock == 1) {
            if (gps_data_ready) {
                gps_data_ready = 0;
                getLocation(&gps_data, gps_buf);
                memset(gps_buf, 0x00, GPSBUF_SIZE);
                gps_active = 1;
                gps_data.date = rolloverDateConvertion(gps_data.date);
                gps_data.speed = gps_data.speed*1.852; // Convert to Km/h

                // Print to terminal log
                char buf[128];
                sprintf(buf, "Longitude: %.2f // Latitude: %.2f // Course: %.2f // Speed: %.2f",
                    gps_data.longi, gps_data.lati, gps_data.course, gps_data.speed);
                printInfo(buf);
            }
        }

        if (read_sensor_cnt >= SENSOR_READ_RATE) {
            read_sensor_cnt = 0;
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 1000);
            waterlevel_reading = HAL_ADC_GetValue(&hadc1);
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 1000);
            adcCh2 = HAL_ADC_GetValue(&hadc1);
            batVol = (15.275/4095)*(float)adcCh2;

            bme280_read_all(&bme280_data);

            water_level = check_water_level(waterlevel_reading);
            if (water_level == WATER_LEVEL_HIGH) {
                // Switch relay to turn water pump on
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
            } else {
                // Switch relay to turn water pump off
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
            }

            // Let alert_system check new values
            bool alert = false;
            alert |= alert_system_check(bme280_data.temperature, TEMPERATURE_ALERT);
            alert |= alert_system_check(bme280_data.humidity, HUMIDITY_ALERT);
            alert |= alert_system_check(batVol, BATTERY_VOLTAGE_ALERT);
            alert |= alert_system_check(water_level, WATER_LEVEL_ALERT);
            alert |= alert_system_check(bme280_data.temperature, TEMPERATURE_ALERT);
            // If any check set alert to true, go to ALERT page on lcd
            if (alert){
                lcd_current_page = LCD_PAGE_ALERTS;
                lcd_update_page = 1;
            }

            // Print new values to terminal log
            char buf[128];
            sprintf(buf, "Temp: %.2f DegC // Hum: %.2f %%RH // Pres: %.2f hPa // BatVol: %.2fV // WaterLvl: %s",
                bme280_data.temperature, bme280_data.humidity, bme280_data.pressure, batVol, waterlevel_str[water_level]);
            printInfo(buf);
	    }

        if (log_data_cnt >= LOG_DATA_RATE) {
            log_data_cnt = 0;
            // Save formatted data to OpenLog
            sprintf(logData, "%02d:%02d:%02d,%06d,%f,%f,%.02f,%.02f,%.02f,%.02f,%.02f,%s,%.02f",
                gps_data.time.hours, gps_data.time.min, gps_data.time.sec, gps_data.date,
                gps_data.lati, gps_data.longi, gps_data.speed, gps_data.course, bme280_data.temperature,
                bme280_data.pressure, bme280_data.humidity, waterlevel_str[water_level],batVol);
            openlogAppendFile("log1.csv", logData);
        }

        if(lcd_update_page) {
            lcd_update_page = 0;
            switch (lcd_current_page)
            {
            case LCD_PAGE_NO_GPS:
                sprintf(lcdBuf, "NO GPS LOCK!");
                printInfo("No GPS lock.");
                lcd_send_string_xy(lcdBuf, 0, 0, CLEAR_LCD);
                break;
            case LCD_PAGE_TIME:
                sprintf(lcdBuf, "Time: %02d:%02d:%02d", gps_data.time.hours, gps_data.time.min , gps_data.time.sec);
                lcd_send_string_xy(lcdBuf, 0, 0, CLEAR_LCD);
                sprintf(lcdBuf, "Date: %06d", gps_data.date);
                lcd_send_string_xy(lcdBuf, 1, 0, DONT_CLEAR_LCD);
                break;
            case LCD_PAGE_GPS:
                sprintf(lcdBuf, "Lati: %.05f", gps_data.lati);
                lcd_send_string_xy(lcdBuf, 0, 0, CLEAR_LCD);
                sprintf(lcdBuf, "Long: %.05f", gps_data.longi);
                lcd_send_string_xy(lcdBuf, 1, 0, DONT_CLEAR_LCD);
                break;
            case LCD_PAGE_SPEED:
                sprintf(lcdBuf, "Km/h: %.02f", gps_data.speed);
                lcd_send_string_xy(lcdBuf, 0, 0, CLEAR_LCD);
                sprintf(lcdBuf, "Heading: %.02f", gps_data.course);
                lcd_send_string_xy(lcdBuf, 1, 0, DONT_CLEAR_LCD);
                break;
            case LCD_PAGE_TEMP:
                sprintf(lcdBuf, "Temp: %.02f DegC", bme280_data.temperature);
                lcd_send_string_xy(lcdBuf, 0, 0, CLEAR_LCD);
                sprintf(lcdBuf, "Pres: %.02f hPa", bme280_data.pressure);
                lcd_send_string_xy(lcdBuf, 1, 0, DONT_CLEAR_LCD);
                break;
            case LCD_PAGE_WATER_LVL:
                sprintf(lcdBuf, "Hum: %.02f %%RH", bme280_data.humidity);
                lcd_send_string_xy(lcdBuf, 0, 0, CLEAR_LCD);
                sprintf(lcdBuf, "WaterLvl: %s", waterlevel_str[water_level]);
                lcd_send_string_xy(lcdBuf, 1, 0, DONT_CLEAR_LCD);
                break;
            case LCD_PAGE_BAT_VOL:
                sprintf(lcdBuf, "BatVol: %.02fV", batVol);
                lcd_send_string_xy(lcdBuf, 0, 0, CLEAR_LCD);
                break;
            case LCD_PAGE_ALERTS:
                sprintf(lcdBuf, "ALERT!!");
                lcd_send_string_xy(lcdBuf, 0, 0, CLEAR_LCD);
                break;
            default:
                break;
            }
        }
        HAL_IWDG_Refresh(&hiwdg);	//Pet the watchdog - will timeout after 10sec
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM15|RCC_PERIPHCLK_ADC1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_HCLK;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PLLCLK_DIV1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 3124;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 40000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 7200-1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 20000-1;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD2_Pin|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PB14 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
//*****ISR FUNCTIONS*****

//Timers interrupt service routine
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    // Timer2
    if (htim == &htim2) {
      systick = 1;
    }

    // Timer15
    if (htim == &htim15) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
    }
}

// External Interrupt ISR Handler CallBackFun
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_4) // INT Source is pin PB4
    {
        if(gps_sat_lock == 0)
        {
            gps_sat_lock = 1;
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART1)
    {
        memcpy(gps_buf, (char*)rx_buf, Size);
        gps_data_ready = 1;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buf, GPSBUF_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

  printInfo("An error occured");
  sprintf(lcdBuf, "ERROR!");
  lcd_send_string_xy(lcdBuf, 0, 0, CLEAR_LCD);
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
