#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

volatile MotorProfile_t motor = {0};
volatile StepperProfile_t nema0 = {0};
volatile StepperProfile_t nema1 = {0};

volatile uint8_t read_flag = 0;
void TIM7_IRQHandler(void) {
  if (TIM7->SR & TIM_SR_UIF) {
    TIM7->SR &= ~TIM_SR_UIF;
  }
  read_flag = 1;
}

void TIM1_UP_TIM10_IRQHandler(void) {
  if (TIM1->SR & TIM_SR_UIF) {
    TIM1->SR &= ~TIM_SR_UIF;
  }

  motor_process_step((MotorProfile_t *)&motor);
  TIM1->ARR = motor.step_delay;
  step_motor((StepperProfile_t *)&nema0);
  step_motor((StepperProfile_t *)&nema1);

  if (motor.state == MOTOR_STATE_STOP) {
    TIM1->CR1 &= ~TIM_CR1_CEN;
  }
}

int main() {
  // Initialize board
  system_init();
  gpio_init();
  uart_init(USART2);
  i2c_init(I2C1);
  timer_master_init();
  timer_isr_init();

  // Initialize hardware
  as_init(I2C1);

  // Configure motion
  GPIOB->ODR |= GPIO_ODR_OD2;
  GPIOB->ODR |= GPIO_ODR_OD1;

  stepper_init((StepperProfile_t *)&nema0, STEPPER_0, 2, 1);
  stepper_init((StepperProfile_t *)&nema1, STEPPER_1, 4, 1);

  master_init((MotorProfile_t *)&motor, 25, 100, 75);
  start_motion((MotorProfile_t *)&motor, TIM1, 12800);

  while (1) {
    if (read_flag) {
      printI(nema0.motor_substeps);
      printS(" ");
      printI(nema1.motor_substeps);
      printS("\r\n");
    }
  }

  return 0;
}