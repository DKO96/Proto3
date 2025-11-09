#ifndef I2C_H
#define I2C_H

#include "stm32f446xx.h"

// Function declarations
void i2c_init(I2C_TypeDef *I2Cx);
void i2c_start(I2C_TypeDef *I2Cx);
void i2c_address(I2C_TypeDef *I2Cx, uint8_t address);
void i2c_stop(I2C_TypeDef *I2Cx);
void i2c_write(I2C_TypeDef *I2Cx, uint8_t *data, uint8_t size);
void i2c_read(I2C_TypeDef *I2Cx, uint8_t *data, uint8_t size);

#endif /* I2C_H */