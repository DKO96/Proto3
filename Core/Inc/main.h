#ifndef MAIN_H_
#define MAIN_H_

// STM32
#include <stdint.h>

// Peripherals
#include "gpio.h"
#include "timer.h"
#include "uart.h"

// Hardware
#include "as5600.h"

// Motor
#include "controller.h"

void system_init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#endif /* MAIN_H_ */