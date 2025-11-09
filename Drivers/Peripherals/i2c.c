#include "i2c.h"

#include "uart.h"

// Function definitions

void i2c_init(I2C_TypeDef *I2Cx) {
  // Enable clock access to I2C
  if (I2Cx == I2C1) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
  } else if (I2Cx == I2C2) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
  }

  // Perform software reset
  I2Cx->CR1 |= I2C_CR1_SWRST;
  I2Cx->CR1 &= ~I2C_CR1_SWRST;

  // Configure peripheral clock frequency PCLK = 45MHz
  I2Cx->CR2 &= ~I2C_CR2_FREQ;
  I2Cx->CR2 |= (45U << I2C_CR2_FREQ_Pos);

  // Select I2C mode (standard/fast)
  I2Cx->CCR |= I2C_CCR_FS;
  I2Cx->CCR &= ~I2C_CCR_DUTY;

  // Configure master mode clock generation (max freq for Sm: 400kHz)
  I2Cx->CCR &= ~I2C_CCR_CCR;  // CCR = 45MHz / (400kHz * 2) = 56Hz
  I2Cx->CCR |= (56U << I2C_CCR_CCR_Pos);

  // Configure maximum rise time (Fm: 300ns)
  I2Cx->TRISE &= ~I2C_TRISE_TRISE;  // TRISE = (Max rise time / PCLK period) + 1
  I2Cx->TRISE |=
      (14U << I2C_TRISE_TRISE_Pos);  // TRISE = (300ns / 22.22ns) + 1 = 14

  // Enable I2C1
  I2Cx->CR1 |= I2C_CR1_PE;
}

void i2c_start(I2C_TypeDef *I2Cx) {
  I2Cx->CR1 |= I2C_CR1_START;
  while (!(I2Cx->SR1 & I2C_SR1_SB))
    ;
}

void i2c_address(I2C_TypeDef *I2Cx, uint8_t address) {
  I2Cx->DR = address;
  while (!(I2Cx->SR1 & I2C_SR1_ADDR))
    ;
}

void i2c_stop(I2C_TypeDef *I2Cx) { I2Cx->CR1 |= I2C_CR1_STOP; }

void i2c_write(I2C_TypeDef *I2Cx, uint8_t *data, uint8_t size) {
  (void)I2Cx->SR1;
  (void)I2Cx->SR2;
  while (size--) {
    while (!(I2Cx->SR1 & I2C_SR1_TXE))
      ;
    I2Cx->DR = *data++;
  }
  while (!(I2Cx->SR1 & I2C_SR1_BTF))
    ;
}

void i2c_read(I2C_TypeDef *I2Cx, uint8_t *data, uint8_t size) {
  if (size == 1) {
    // Single byte receive
    (void)I2Cx->SR1;
    (void)I2Cx->SR2;            // clear ADDR
    I2Cx->CR1 &= ~I2C_CR1_ACK;  // disable ACK to send NACK
    while (!(I2Cx->SR1 & I2C_SR1_BTF))
      ;                // wait to receive data (DR not empty)
    *data = I2Cx->DR;  // read data
    i2c_stop(I2Cx);
  } else if (size == 2) {
    // 2 byte receive
    I2Cx->CR1 &= ~I2C_CR1_ACK;
    I2Cx->CR1 |= I2C_CR1_POS;
    (void)I2Cx->SR1;
    (void)I2Cx->SR2;  // clear ADDR
    while (!(I2Cx->SR1 & I2C_SR1_BTF))
      ;
    i2c_stop(I2Cx);
    data[0] = I2Cx->DR;
    data[1] = I2Cx->DR;
  }
}