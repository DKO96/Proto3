#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#define NUM_MOTORS 3

volatile MotorProfile_t motor = {0};
volatile StepperProfile_t motors[NUM_MOTORS] = {0};

void TIM1_UP_TIM10_IRQHandler(void) {
  if (TIM1->SR & TIM_SR_UIF) {
    TIM1->SR &= ~TIM_SR_UIF;
  }

  motor_process_step((MotorProfile_t *)&motor);
  TIM1->ARR = motor.step_delay;

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    step_motor((StepperProfile_t *)&motors[i]);
  }

  if (motor.state == MOTOR_STATE_STOP) {
    TIM1->CR1 &= ~TIM_CR1_CEN;
  }
}

int main() {
  // Initialize stm32
  system_init();
  gpio_init();
  uart_init(USART2);

  // Initialize stepper motor
  StepperPins_t motor_pins[] = {
      {.step = {GPIOA, 0}, .dir = {GPIOB, 2}},
      {.step = {GPIOA, 1}, .dir = {GPIOB, 1}},
      {.step = {GPIOA, 4}, .dir = {GPIOB, 15}},
  };

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    stepper_init((StepperProfile_t *)&motors[i], &motor_pins[i]);
  }

  // Configure stepper
  configure_stepper((StepperProfile_t *)&motors[0], 0, 1, 1);
  configure_stepper((StepperProfile_t *)&motors[1], 0, 5, 3);
  configure_stepper((StepperProfile_t *)&motors[2], 0, 5, 1);

  // Start move
  timer_master_init();
  master_init((MotorProfile_t *)&motor, 25, 65, 75);
  start_motion((MotorProfile_t *)&motor, TIM1, 16000);

  while (1) {
  }

  return 0;
}