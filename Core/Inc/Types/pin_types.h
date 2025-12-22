#ifndef PIN_TYPES_H
#define PIN_TYPES_H

#include "stm32f446xx.h"

/**
 * @brief Single GPIO pin configuration
 */
typedef struct {
  GPIO_TypeDef *port;
  uint8_t pin;
} GpioPin_t;

/**
 * @brief Stepper motor pin configuration (step + direction)
 */
typedef struct {
  GpioPin_t step;
  GpioPin_t dir;
  float init_rad;
} StepperPinConfig_t;

// GPIO helper macros
#define GPIO_PIN_SET_MASK(pin) (1UL << (pin))
#define GPIO_PIN_RESET_MASK(pin) (1UL << ((pin) + 16))

#endif /* PIN_TYPES_H */