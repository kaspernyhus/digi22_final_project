/**
 * @file i2c.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-22
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>

void i2c_init(void);
void i2c_write(uint8_t address, uint8_t* data, uint8_t bytes);
void i2c_read(uint8_t address, uint8_t reg, uint8_t* data, uint8_t bytes_to_read);

#endif /* I2C_H_ */
