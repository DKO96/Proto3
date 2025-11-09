#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>
#include "i2c.h"

// Function declarations
void as_write(I2C_TypeDef *I2Cx, uint8_t address, uint8_t register_addr, uint8_t *data, uint8_t size);
void as_read(I2C_TypeDef *I2Cx, uint8_t address, uint8_t register_addr, uint8_t *buffer, uint8_t size);
void as_init(I2C_TypeDef *I2Cx);
uint8_t as_status(I2C_TypeDef *I2Cx);
uint16_t as_angle(I2C_TypeDef *I2Cx);

#endif /* AS5600_H */