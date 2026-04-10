#include "main.h"

#include <math.h>

RobotHandle_t robot = {0};

void EXTI15_10_IRQHandler(void) {
  if (EXTI->PR & EXTI_PR_PR13) {
    EXTI->PR |= EXTI_PR_PR13;
  }

  if (robot.safety == SAFETY_STATE_ON) {
    robot.safety = SAFETY_STATE_OFF;
    LED_OFF();
  } else {
    robot.safety = SAFETY_STATE_ON;
    LED_ON();
    TIM1->CR1 &= ~TIM_CR1_CEN;
  }
}

void TIM1_UP_TIM10_IRQHandler(void) {
  TIM1->SR &= ~TIM_SR_UIF;

  robot.motion.current_step++;
  if (robot.motion.current_step >= robot.motion.total_steps) {
    robot.motion.state = MOTION_STATE_IDLE;
    TIM1->CR1 &= ~TIM_CR1_CEN;
    printS("\r\nStop timer ISR\r\n");
    delay_ms(500);
  }

  for (uint8_t i = 0; i < ROBOT_NUM_JOINTS; i++) {
    if (robot.joints[i].sync_numerator > 0) {
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
}

int main() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  GPIOA->MODER |= (1 << 10);  // PA5 output
  GPIOA->ODR |= (1 << 5);     // LED on

  /* Initialize hardware */
  system_init();
  gpio_init();
  // exti_init();
  uart_init(USART2);
  // timer_master_init();

  printS("Initializing robot hardware...\r\n");

  /* Initialize robot hardware */
  const StepperPinConfig_t stepper_pins[ROBOT_NUM_JOINTS] = {
      {.step = {GPIOA, 0}, .dir = {GPIOB, 2}, .init_rad = 0.0f},
      {.step = {GPIOA, 1}, .dir = {GPIOB, 1}, .init_rad = -1.899f},
      {.step = {GPIOA, 4}, .dir = {GPIOB, 15}, .init_rad = 2.585f},
  };
  robot_init(&robot, stepper_pins, LINK_1, LINK_2);

  // TIM1->ARR = 9999;

  while (1) {
    CartesianPoint_t target = {.x = 0.0f, .y = 0.0f, .z = 185.0f};
    float s = 1.0f;  // desired spacing between waypoints [mm]

    // Current angle of joints
    // float angle0 = calculate_angle(&robot.joints[0]);
    // float angle1 = calculate_angle(&robot.joints[1]);
    // float angle2 = calculate_angle(&robot.joints[2]);

    printS("test1\r\n");
    float angle0 = calculate_angle(&robot.joints[0]);
    printS("test2\r\n");
    float val = angle0 * 1000.0f;
    printS("test3\r\n");
    int ival = (int)val;
    printS("test4\r\n");
    printI(ival);
    printS("\r\n");

    // Current position of end-effector (forward kinematics)
    // float x = LINK_1 * cosf(angle1) + LINK_2 * cosf(angle1 + angle2);
    // printS("hello\r\n");

    // x = (LINK_1 * cosf(angle1) + LINK_2 * cosf(angle1 + angle2)) *
    // cosf(angle0); float y =
    //     (LINK_1 * cosf(angle1) + LINK_2 * cosf(angle1 + angle2)) *
    //     sinf(angle0);
    // float z = LINK_1 * sinf(angle1) + LINK_2 * sinf(angle1 + angle2);

    delay_ms(1000);
  }

  return 0;
}