#include "main.h"

#include <math.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

RobotHandle_t robot = {0};
QueueHandle_t waypoint_queue;
QueueHandle_t ik_queue;
SemaphoreHandle_t motion_complete_semphr;

void EXTI15_10_IRQHandler(void) {
  if (EXTI->PR & EXTI_PR_PR13) {
    EXTI->PR |= EXTI_PR_PR13;
  }

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (robot.safety == SAFETY_STATE_ON) {
    robot.safety = SAFETY_STATE_OFF;
    LED_OFF();
    xSemaphoreGiveFromISR(motion_complete_semphr, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  } else {
    robot.safety = SAFETY_STATE_ON;
    LED_ON();
    TIM1->CR1 &= ~TIM_CR1_CEN;
  }
}

void TIM1_UP_TIM10_IRQHandler(void) {
  TIM1->SR &= ~TIM_SR_UIF;

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  motion_process_step(&robot.motion);
  TIM1->ARR = robot.motion.step_delay;

  for (uint8_t i = 0; i < ROBOT_NUM_JOINTS; i++) {
    // stepper_sync_step(&robot.joints[i]);

    // DEBUG: desync stepper motors
    if (robot.joints[i].sync_numerator >= 0) {
      /* Generate step pulse */
      robot.joints[i].step_port->BSRR = robot.joints[i].step_set;
      delay_us(1);
      robot.joints[i].step_port->BSRR = robot.joints[i].step_reset;

      robot.joints[i].sync_numerator--;

      /* Update position */
      if (robot.joints[i].direction) {
        robot.joints[i].position_steps++;
      } else {
        robot.joints[i].position_steps--;
      }
    }
  }

  if (robot.motion.state == MOTION_STATE_IDLE) {
    TIM1->CR1 &= ~TIM_CR1_CEN;
    xSemaphoreGiveFromISR(motion_complete_semphr, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

static void waypoint_task(void *pvParameters) {
  CartesianPoint_t waypoints[] = {
      {.x = 100.0f, .y = 0.0f, .z = 85.0f},
      // {.x = 90.0f, .y = -65.0f, .z = 0.0f},
      // {.x = 90.0f, .y = 65.0f, .z = 0.0f},
      // {.x = 90.0f, .y = 65.0f, .z = 130.0f},
      // {.x = 90.0f, .y = -65.0f, .z = 130.0f},
  };

  const size_t num_waypoints = sizeof(waypoints) / sizeof(waypoints[0]);

  for (;;) {
    for (uint8_t i = 0; i < num_waypoints; i++) {
      xQueueSend(waypoint_queue, &waypoints[i], portMAX_DELAY);
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

static void trajectory_task(void *pvParameters) {
  CartesianPoint_t target;
  JointAngles_t angles;
  // desired spacing between trajectory points
  float s = 1.0f;

  for (;;) {
    xQueueReceive(waypoint_queue, &target, portMAX_DELAY);
    // Trajectory generation
    float tx = target.x;
    float ty = target.y;
    float tz = target.z;

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    // Current angle of joints
    float angle0 = calculate_angle(&robot.joints[0]);
    float angle1 = calculate_angle(&robot.joints[1]);
    float angle2 = calculate_angle(&robot.joints[2]);

    // Current position of end-effector (forward kinematics)
    x = (LINK_1 * cosf(angle1) + LINK_2 * cosf(angle1 + angle2)) * cosf(angle0);
    y = (LINK_1 * cosf(angle1) + LINK_2 * cosf(angle1 + angle2)) * sinf(angle0);
    z = LINK_2 * sinf(angle1) + LINK_2 * sinf(angle1 + angle2);

    // Determine trajectory points (euclidean distance)
    float dx = tx - x;
    float dy = ty - y;
    float dz = tz - z;

    float l2_norm = sqrtf(dx * dx + dy * dy + dz * dz);

    // Determine unit vector
    float ux = dx / l2_norm;
    float uy = dy / l2_norm;
    float uz = dz / l2_norm;

    // Determine number of points along trajectory line
    int N = (int)ceilf(l2_norm / s);

    // Generate point along trajectory
    for (int i = 0; i < N; i++) {
      float distance = i * s;

      if (distance > l2_norm) {
        distance = l2_norm;
      }

      CartesianPoint_t point;
      point.x = x + ux * distance;
      point.y = y + uy * distance;
      point.z = z + uz * distance;
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

    if (robot.safety == SAFETY_STATE_ON) {
      xSemaphoreGive(motion_complete_semphr);
      continue;
    }

    plan = robot_plan_motion(&robot, &target_angles);

    if (!plan.valid) {
      /* No motion required (already at target) */
      xSemaphoreGive(motion_complete_semphr);
      continue;
    }

    robot_execute_plan(&robot, &plan, TIM1);
  }
}

static void monitor_task(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    robot_monitor(&robot);
    xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
  }
}

int main() {
  /* Initialize hardware */
  system_init();
  gpio_init();
  exti_init();
  uart_init(USART2);
  timer_master_init();

  /* Initialize robot hardware */
  const StepperPinConfig_t stepper_pins[ROBOT_NUM_JOINTS] = {
      {.step = {GPIOA, 0}, .dir = {GPIOB, 2}, .init_rad = 0.0f},     // joint 0
      {.step = {GPIOA, 1}, .dir = {GPIOB, 1}, .init_rad = -1.899f},  // joint 1
      {.step = {GPIOA, 4}, .dir = {GPIOB, 15}, .init_rad = 2.585f},  // joint 2
  };
  robot_init(&robot, stepper_pins, LINK_1, LINK_2);

  /* Initialize rtos */
  waypoint_queue = xQueueCreate(WAYPOINT_QUEUE, sizeof(CartesianPoint_t));
  ik_queue = xQueueCreate(INV_KIN_QUEUE, sizeof(JointAngles_t));

  motion_complete_semphr = xSemaphoreCreateBinary();

  xTaskCreate(waypoint_task, "Waypoint", 1000, NULL, WAYPOINT_PRIORITY, NULL);
  // xTaskCreate(ik_task, "IK", 1000, NULL, INV_KIN_PRIORITY, NULL);
  xTaskCreate(trajectory_task, "IK", 1000, NULL, INV_KIN_PRIORITY, NULL);
  xTaskCreate(motor_task, "Motor", 1000, NULL, MOTOR_PRIORITY, NULL);
  // xTaskCreate(monitor_task, "Monitor", 1000, NULL, MONITOR_PRIORITY, NULL);

  /* Start scheduler */
  vTaskStartScheduler();

  return 0;
}