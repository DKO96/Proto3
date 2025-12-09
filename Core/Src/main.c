#include "main.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

RobotHandle_t robot;
QueueHandle_t waypoint_queue;
QueueHandle_t ik_queue;
SemaphoreHandle_t motion_complete_semphr;

void TIM1_UP_TIM10_IRQHandler(void) {
  TIM1->SR &= ~TIM_SR_UIF;

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  motion_process_step(&robot.motion);
  TIM1->ARR = robot.motion.step_delay;

  for (uint8_t i = 0; i < ROBOT_NUM_JOINTS; i++) {
    stepper_sync_step(&robot.joints[i]);
  }

  if (robot.motion.state == MOTION_STATE_IDLE) {
    TIM1->CR1 &= ~TIM_CR1_CEN;
    xSemaphoreGiveFromISR(motion_complete_semphr, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

static void waypoint_task(void *pvParameters) {
  CartesianPoint_t waypoints[] = {
      {.x = 100.0f, .y = 0.0f, .z = 52.0f},
      {.x = -100.0f, .y = 0.0f, .z = 52.0f},
      // {.x = 80.0f, .y = 0.0f, .z = 75.0f},
      // {.x = -100.0f, .y = 0.0f, .z = 25.0f},
  };

  const size_t num_waypoints = sizeof(waypoints) / sizeof(waypoints[0]);

  for (;;) {
    for (uint8_t i = 0; i < num_waypoints; i++) {
      xQueueSend(waypoint_queue, &waypoints[i], portMAX_DELAY);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

static void ik_task(void *pvParameters) {
  CartesianPoint_t target;
  JointAngles_t angles;

  for (;;) {
    xQueueReceive(waypoint_queue, &target, portMAX_DELAY);

    angles = robot_inverse_kinematics(&robot, &target);

    if (!angles.valid) {
      // TODO: handle unreachable target
      continue;
    }

    xQueueSend(ik_queue, &angles, portMAX_DELAY);
  }
}

static void motor_task(void *pvParameters) {
  JointAngles_t target_angles;
  MotionPlan_t plan;

  for (;;) {
    xQueueReceive(ik_queue, &target_angles, portMAX_DELAY);
    xSemaphoreTake(motion_complete_semphr, portMAX_DELAY);

    plan = robot_plan_motion(&robot, &target_angles);

    if (!plan.valid) {
      /* No motion required (already at target) */
      xSemaphoreGive(motion_complete_semphr);
      continue;
    }

    robot_execute_plan(&robot, &plan, TIM1);
  }
}

int main() {
  /* Initialize hardware */
  system_init();
  gpio_init();
  uart_init(USART2);
  timer_master_init();

  /* Initialize robot hardware */
  const StepperPinConfig_t stepper_pins[ROBOT_NUM_JOINTS] = {
      {.step = {GPIOA, 0}, .dir = {GPIOB, 2}},   // joint 0
      {.step = {GPIOA, 1}, .dir = {GPIOB, 1}},   // joint 1
      {.step = {GPIOA, 4}, .dir = {GPIOB, 15}},  // joint 2
  };
  robot_init(&robot, stepper_pins, LINK_1, LINK_2);

  /* Initialize rtos */
  waypoint_queue = xQueueCreate(3, sizeof(CartesianPoint_t));
  ik_queue = xQueueCreate(2, sizeof(JointAngles_t));

  motion_complete_semphr = xSemaphoreCreateBinary();
  xSemaphoreGive(motion_complete_semphr);

  xTaskCreate(waypoint_task, "Waypoint", 1000, NULL, 1, NULL);
  xTaskCreate(ik_task, "IK", 1000, NULL, 2, NULL);
  xTaskCreate(motor_task, "Motor", 1000, NULL, 3, NULL);

  /* Start scheduler */
  vTaskStartScheduler();

  return 0;
}