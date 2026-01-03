#ifndef TIMER_H
#define TIMER_H

#include "stm32f446xx.h"

/**
 * @brief Initialize hardware timer 1 for stepper motor control
 */
void timer_master_init(void);

#endif /* TIMER_H */
