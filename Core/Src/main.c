#include "main.h"

#include <math.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#define NUM_MOTORS 3
#define MAX_SPEED 26
#define ACCELERATION 75
#define MIN_DELAY 75
#define LINK_1 52.0f
#define LINK_2 100.0f

typedef struct {
  float x;
  float y;
  float z;
} WaypointData_t;

typedef struct {
  float t[3];
} AngleData_t;

volatile MotorProfile_t motor = {0};
volatile StepperProfile_t motors[NUM_MOTORS] = {0};
QueueHandle_t xWaypointQueue;
QueueHandle_t xIKQueue;
SemaphoreHandle_t xMotorSemaphore;

volatile uint32_t isr_count = 0;
void TIM1_UP_TIM10_IRQHandler(void) {
  if (TIM1->SR & TIM_SR_UIF) {
    TIM1->SR &= ~TIM_SR_UIF;
  }

  // printI(isr_count);
  // printS("\r\n");
  isr_count++;

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  motor_process_step((MotorProfile_t *)&motor);
  TIM1->ARR = motor.step_delay;

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    step_motor((StepperProfile_t *)&motors[i]);
  }

  if (motor.state == MOTOR_STATE_STOP) {
    TIM1->CR1 &= ~TIM_CR1_CEN;
    xSemaphoreGiveFromISR(xMotorSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    printS("stop motors\r\n");
  }
}

void vWaypointTask(void *pvParameters) {
  WaypointData_t waypoints[] = {
      {100.0f, 50.0f, 52.0f},
      // {-100.0f, 30.0f, 60.0f},
  };

  for (;;) {
    for (uint8_t i = 0; i < sizeof(waypoints) / sizeof(waypoints[0]); i++) {
      xQueueSend(xWaypointQueue, &waypoints[i], portMAX_DELAY);
    }
  }
}

void vIKTask(void *pvParameters) {
  WaypointData_t xReceivedWaypoint;
  float x, y, z;
  AngleData_t angles;

  for (;;) {
    xQueueReceive(xWaypointQueue, &xReceivedWaypoint, portMAX_DELAY);

    x = xReceivedWaypoint.x;
    y = xReceivedWaypoint.y;
    z = xReceivedWaypoint.z;

    float r = sqrtf(x * x + y * y);
    float cos_t2 = (r * r + z * z - LINK_1 * LINK_1 - LINK_2 * LINK_2) /
                   (2 * LINK_1 * LINK_2);

    if (fabsf(cos_t2) > 1) continue;

    angles.t[2] = -acosf(cos_t2);

    angles.t[1] = atan2f(z, r) - atan2f(LINK_2 * sinf(angles.t[2]),
                                        LINK_1 + LINK_2 * cosf(angles.t[2]));

    angles.t[0] = atan2f(y, x);

    xQueueSend(xIKQueue, &angles, portMAX_DELAY);
  }
}

void vMotorTask(void *pvParameters) {
  AngleData_t xReceivedAngle;
  int steps[NUM_MOTORS];

  for (;;) {
    xQueueReceive(xIKQueue, &xReceivedAngle, portMAX_DELAY);
    xSemaphoreTake(xMotorSemaphore, portMAX_DELAY);
    int max_steps = 0;

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      steps[i] =
          angle_to_steps((StepperProfile_t *)&motors[i], xReceivedAngle.t[i]);

      if (abs(steps[i]) > max_steps) {
        max_steps = abs(steps[i]);
      }
    }

    if (max_steps == 0) {
      xSemaphoreGive(xMotorSemaphore);
      continue;
    }

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      uint8_t direction = (steps[i] >= 0) ? 1 : 0;
      int slave_ratio = abs(steps[i]);
      int master_ratio = max_steps;

      // printI(slave_ratio);
      // printS("\t");
      // printI(master_ratio);
      // printS("\r\n");

      configure_stepper((StepperProfile_t *)&motors[i], direction, master_ratio,
                        slave_ratio);
    }

    // Start move
    master_init((MotorProfile_t *)&motor, MAX_SPEED, ACCELERATION, MIN_DELAY);
    start_motion((MotorProfile_t *)&motor, TIM1, max_steps);
  }
}

int main() {
  // Initialize stm32
  system_init();
  gpio_init();
  uart_init(USART2);
  timer_master_init();

  // Initialize stepper motor
  StepperPins_t motor_pins[] = {
      {.step = {GPIOA, 0}, .dir = {GPIOB, 2}},
      {.step = {GPIOA, 1}, .dir = {GPIOB, 1}},
      {.step = {GPIOA, 4}, .dir = {GPIOB, 15}},
  };

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    stepper_init((StepperProfile_t *)&motors[i], &motor_pins[i]);
  }

  xWaypointQueue = xQueueCreate(3, sizeof(WaypointData_t));
  xIKQueue = xQueueCreate(2, sizeof(AngleData_t));

  xMotorSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(xMotorSemaphore);

  xTaskCreate(vWaypointTask, "Waypoint", 1000, NULL, 1, NULL);
  xTaskCreate(vIKTask, "IK", 1000, NULL, 2, NULL);
  xTaskCreate(vMotorTask, "Motor", 1000, NULL, 3, NULL);

  vTaskStartScheduler();

  return 0;
}