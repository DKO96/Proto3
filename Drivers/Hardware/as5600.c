#include "as5600.h"

#define AS_ADDR 0x36
#define AS_STATUS 0x0B
#define AS_CONF 0x07
#define ANGLE 0x0E
#define MD (1 << 5)
#define ML (1 << 4)
#define MH (1 << 3)

void as_write(I2C_TypeDef *I2Cx, uint8_t address, uint8_t register_addr,
              uint8_t *data, uint8_t size) {
  i2c_start(I2Cx);
  i2c_address(I2Cx, (address << 1) | 0x00);
  i2c_write(I2Cx, &register_addr, 1);

  // Master transmitter mode
  i2c_write(I2Cx, data, size);
  i2c_stop(I2Cx);
}

void as_read(I2C_TypeDef *I2Cx, uint8_t address, uint8_t register_addr,
             uint8_t *buffer, uint8_t size) {
  i2c_start(I2Cx);
  i2c_address(I2Cx, (address << 1) | 0x00);
  i2c_write(I2Cx, &register_addr, 1);

  // Master receiver mode
  i2c_start(I2Cx);
  i2c_address(I2Cx, (address << 1) | 0x01);
  i2c_read(I2Cx, buffer, size);
}

void as_init(I2C_TypeDef *I2Cx) {
  uint8_t settings = 0x1F;
  as_write(I2Cx, AS_ADDR, AS_CONF, &settings, 1);
}

uint8_t as_status(I2C_TypeDef *I2Cx) {
  uint8_t status;
  as_read(I2Cx, AS_ADDR, AS_STATUS, &status, 1);
  status &= 0x38;
  return status;
}

uint16_t as_angle(I2C_TypeDef *I2Cx) {
  uint8_t buffer[2];
  as_read(I2Cx, AS_ADDR, ANGLE, buffer, 2);
  return (buffer[0] << 8 | buffer[1]) & 0x0FFF;
}