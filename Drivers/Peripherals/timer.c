#include "timer.h"

/*
  Configure TIM5 Channel 2 for output compare mode
*/

// Function definitions
void timer_pwm_init(TIM_TypeDef *TIMx) {
  // Enable clock
  if (TIMx == TIM2) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  } else if (TIMx == TIM5) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
  }

  // Configure prescaler: 1MHz
  TIMx->PSC = 89;

  // Configure auto-reload: 75us
  TIMx->ARR = 999;

  // Configure capture/compare: 1us
  TIMx->CCR2 = 1;

  // Configure PWM mode 1
  TIMx->CCMR1 &= ~TIM_CCMR1_OC2M;
  TIMx->CCMR1 |= (6U << TIM_CCMR1_OC2M_Pos);

  // Enable output compare preload for channel 2
  TIMx->CCMR1 |= TIM_CCMR1_OC2PE;

  // Enable auto-reload preload
  TIMx->CR1 |= TIM_CR1_ARPE;

  // Configure channel 2 output polarity (active high)
  TIMx->CCER &= ~TIM_CCER_CC2P;

  // Enable channel 2 output
  TIMx->CCER |= TIM_CCER_CC2E;

  // Set counter direction (upcounting)
  TIMx->CR1 &= ~TIM_CR1_DIR;

  // Generate update event
  TIMx->EGR |= TIM_EGR_UG;

  // Enable interrupt on capture/compare
  TIMx->DIER |= TIM_DIER_CC2IE;

  // Configure TIM5 interrupt in NVIC
  NVIC_SetPriority(TIM5_IRQn, 2);
  NVIC_EnableIRQ(TIM5_IRQn);
}

void timer_isr_init(void) {
  // Enable clock access for TIM7
  RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

  // Configure interrupt frequency
  TIM7->PSC = 89;
  TIM7->ARR = 99;

  // Enable TIM7 and interrupt
  TIM7->DIER |= TIM_DIER_UIE;
  TIM7->CR1 |= TIM_CR1_CEN;

  // Enable TIM7 interrupt in NVIC
  NVIC_SetPriority(TIM7_IRQn, 1);
  NVIC_EnableIRQ(TIM7_IRQn);
}
