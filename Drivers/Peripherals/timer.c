#include "timer.h"

void timer_master_init(void) {
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  TIM1->PSC = 179;
  TIM1->ARR = 999;

  TIM1->CR1 |= TIM_CR1_ARPE;
  TIM1->EGR |= TIM_EGR_UG;
  TIM1->SR &= ~TIM_SR_UIF;
  TIM1->DIER |= TIM_DIER_UIE;

  // Configure TIM1 interrupt in NVIC
  NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 6);
  NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}