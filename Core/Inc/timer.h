#ifndef TIMER_H
#define TIMER_H

#include "stm32f446xx.h"

// Function declarations
void timer_master_init(void);
void timer_pwm_init(TIM_TypeDef *TIMx);
void timer_isr_init(void);

#endif /* TIMER_H */
