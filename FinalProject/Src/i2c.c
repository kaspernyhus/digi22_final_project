/**
 * @file i2c.c
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-22
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "i2c.h"
#include <stdio.h>
#include "stm32f30x_conf.h" // STM32 config


// I2C, PB6: I2C1_SCL, PB7: I2C1_SDA. AF4
void i2c_init(void) {
	// Enable clocks to peripherals
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);


	// I2C1 SDA and SCL pin configuration
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; // Open drain?
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // Pull-ups on the BME280 board (10k)?
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_4);

	/* I2C1 Reset */
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

	// I2C1 configuration
	I2C_InitTypeDef I2C_InitStructure;
	I2C_StructInit(&I2C_InitStructure);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_Timing = 0x2000090E; // MX
    I2C_Init(I2C1, &I2C_InitStructure);
    I2C_Cmd(I2C1, ENABLE);

	printf("I2C initialized\n");
}

/**
 * @brief WRITE
 *
 * @param address
 * @param data
 * @param bytes
 */
void i2c_write(uint8_t address, uint8_t* data, uint8_t bytes)
{
	I2C_TransferHandling(I2C1,(address<<1),bytes,I2C_AutoEnd_Mode,I2C_Generate_Start_Write);

	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET); // wait for TXE = 1 -> Tx reg empty

	while(bytes) {
		I2C_SendData(I2C1, *data++);
		while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXE) == RESET); // Transmit data register empty
		bytes--;
	}
	// I2C_GenerateSTOP(I2C1, ENABLE);

	// while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TC) == RESET);

	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF) == RESET);
	I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);
}

/**
 * @brief READ
 *
 * @param address
 * @param reg
 * @param data
 * @param bytes_to_read
 */
void i2c_read(uint8_t address, uint8_t reg, uint8_t* data, uint8_t bytes_to_read)
{
    uint8_t addr_shifted = address << 1;

    // Wait if bus is busy
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);

    // Send address and reg
    I2C_TransferHandling(I2C1,addr_shifted,1,I2C_SoftEnd_Mode,I2C_Generate_Start_Write);
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET);
    I2C_SendData(I2C1, reg);
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TC) == RESET);

    // Receive data
    I2C_TransferHandling(I2C1,addr_shifted,bytes_to_read,I2C_AutoEnd_Mode,I2C_Generate_Start_Read);
    while(bytes_to_read > 0) {
        // wait on receive data register to be not empty
        while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET);
        *data = I2C_ReceiveData(I2C1);
        data++;
        bytes_to_read--;
    }

    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF) == RESET);
	I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);
}
