#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

volatile MotorProfile_t motor = {0};
volatile StepperProfile_t nema1 = {0};
volatile StepperProfile_t nema2 = {0};
volatile StepperProfile_t nema3 = {0};

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

  step_motor((StepperProfile_t *)&nema1);
  step_motor((StepperProfile_t *)&nema2);
  step_motor((StepperProfile_t *)&nema3);

  if (motor.state == MOTOR_STATE_STOP) {
    TIM1->CR1 &= ~TIM_CR1_CEN;
  }
}

int main() {
  // Initialize stm32
  system_init();
  gpio_init();
  uart_init(USART2);
  timer_master_init();
  timer_isr_init();

  // Initialize stepper motor
  stepper_init((StepperProfile_t *)&nema1, (GPIO_TypeDef *)GPIOA,
               (GPIO_TypeDef *)GPIOB, GPIO_BSRR_BS0, GPIO_BSRR_BR0,
               GPIO_ODR_OD2);
  stepper_init((StepperProfile_t *)&nema2, (GPIO_TypeDef *)GPIOA,
               (GPIO_TypeDef *)GPIOB, GPIO_BSRR_BS1, GPIO_BSRR_BR1,
               GPIO_ODR_OD1);
  stepper_init((StepperProfile_t *)&nema3, (GPIO_TypeDef *)GPIOA,
               (GPIO_TypeDef *)GPIOB, GPIO_BSRR_BS4, GPIO_BSRR_BR4,
               GPIO_ODR_OD15);

  // Configure stepper
  nema1.GPIO_DIR->ODR |= nema1.pin_dir_mask;
  configure_stepper((StepperProfile_t *)&nema1, 1, 1);

  // nema2.GPIO_DIR->ODR |= nema2.pin_dir_mask;
  nema2.GPIO_DIR->ODR |= nema2.pin_dir_mask;
  configure_stepper((StepperProfile_t *)&nema2, 1, 1);

  nema3.GPIO_DIR->ODR |= nema3.pin_dir_mask;
  configure_stepper((StepperProfile_t *)&nema3, 1, 0);

  // Start move
  // master_init((MotorProfile_t *)&motor, 25, 65, 75);
  master_init((MotorProfile_t *)&motor, 4, 350, 500);
  start_motion((MotorProfile_t *)&motor, TIM1, 200);

  while (1) {
    if (read_flag) {
      printI(nema1.motor_step_count);
      printS(" ");
      printI(nema2.motor_step_count);
      printS(" ");
      // printI(nema3.motor_step_count);
      printS("\r\n");
    }
  }

  return 0;
}