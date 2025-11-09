#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

volatile MotorProfile_t motor = {0};
void TIM5_IRQHandler(void) {
  if (TIM5->SR & TIM_SR_CC2IF) {
    TIM5->SR &= ~TIM_SR_CC2IF;
  }

  motor_process_step((MotorProfile_t *)&motor);

  TIM5->ARR = motor.step_delay;

  if (motor.state == MOTOR_STATE_STOP) {
    TIM5->CR1 &= ~TIM_CR1_CEN;
  }
}

volatile uint8_t read_flag = 0;
void TIM7_IRQHandler(void) {
  if (TIM7->SR & TIM_SR_UIF) {
    TIM7->SR &= ~TIM_SR_UIF;
  }
  read_flag = 1;
}

int main() {
  // Initialize board
  system_init();
  gpio_init();
  uart_init(USART2);
  i2c_init(I2C1);
  timer_pwm_init(TIM5);
  timer_isr_init();

  // Initialize hardware
  as_init(I2C1);

  // Configure motion
  GPIOB->ODR |= GPIO_ODR_OD2;

  motor_init((MotorProfile_t *)&motor, 30, 60, 74);

  start_motion((MotorProfile_t *)&motor, TIM5, 6400);

  while (1) {
    if (read_flag) {
      printI(motor.step_count);
      printS("\r\n");
    }
  }

  return 0;
}