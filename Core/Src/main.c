#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

volatile MotorProfile_t motor = {0};
volatile int motor_substeps = 0;
volatile int motor_ratio_master = 3;
volatile int motor_ratio_slave = 2;
volatile int motor_accumulator = 0;

void step_motor(void) {
  GPIOA->BSRR = GPIO_BSRR_BS4;
  delay_us(1);
  GPIOA->BSRR = GPIO_BSRR_BR4;
}

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

  motor_accumulator += motor_ratio_slave;

  if (motor_accumulator >= motor_ratio_master) {
    step_motor();
    motor_substeps++;
    motor_accumulator -= motor_ratio_master;
  }

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

  motor_init((MotorProfile_t *)&motor, 25, 100, 75);
  start_motion((MotorProfile_t *)&motor, TIM1, 4800);

  while (1) {
    if (read_flag) {
      printI(motor_substeps);
      printS("\r\n");
    }
  }

  return 0;
}