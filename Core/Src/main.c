#include "main.h"

#include <stdlib.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#define NUM_MOTORS 3
#define MAX_SPEED 25
#define ACCELERATION 75
#define MIN_DELAY 75

volatile MotorProfile_t motor = {0};
volatile StepperProfile_t motors[NUM_MOTORS] = {0};
QueueHandle_t xUARTQueue;

void USART2_IRQHandler(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (USART2->SR & USART_SR_RXNE) {
    uint8_t ch = USART2->DR;
    xQueueSendFromISR(xUARTQueue, &ch, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

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

void vMotorTask(void *pvParameters) {
  uint8_t receivedChar;
  char buffer[100];
  int bufferIndex = 0;
  int value;

  for (;;) {
    xQueueReceive(xUARTQueue, &receivedChar, portMAX_DELAY);

    // Checks for end of line
    if (receivedChar == '\r' || receivedChar == '\n') {
      buffer[bufferIndex] = '\0';
      printS("\r\n");

      value = atoi(buffer);

      // Configure stepper
      configure_stepper((StepperProfile_t *)&motors[0], 0, 1, 1);
      configure_stepper((StepperProfile_t *)&motors[1], 0, 5, 3);
      configure_stepper((StepperProfile_t *)&motors[2], 0, 5, 1);

      // Start move
      timer_master_init();
      master_init((MotorProfile_t *)&motor, MAX_SPEED, ACCELERATION, MIN_DELAY);
      start_motion((MotorProfile_t *)&motor, TIM1, value);

      bufferIndex = 0;
      continue;
    }

    usart2_write(receivedChar);
    if (bufferIndex < sizeof(buffer) - 1) {
      buffer[bufferIndex++] = receivedChar;
    }
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

  xUARTQueue = xQueueCreate(100, sizeof(uint8_t));
  xTaskCreate(vMotorTask, "Motor", 1000, NULL, 1, NULL);

  vTaskStartScheduler();

  while (1) {
  }

  return 0;
}