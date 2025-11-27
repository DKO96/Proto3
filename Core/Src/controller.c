#include "controller.h"

#include <math.h>

#include "main.h"

#define INIT_CONST 0.676
#define TIMER_FREQ 1000000
#define ALPHA 0.00196

void master_init(MotorProfile_t *motor, float max_speed, uint16_t accel_rate,
                 uint16_t min_delay) {
  motor->max_speed = max_speed;
  motor->accel_rate = accel_rate;
  motor->min_delay = min_delay;
}

static uint16_t calculate_initial_delay(MotorProfile_t *motor) {
  float initial_step =
      INIT_CONST * TIMER_FREQ * sqrt(2 * ALPHA / motor->accel_rate);
  return (uint16_t)initial_step;
}

static void calculate_motion_profile(MotorProfile_t *motor) {
  float steps_to_run = (motor->max_speed * motor->max_speed) /
                       (2.0f * ALPHA * motor->accel_rate);

  float steps_to_decel = motor->total_steps - steps_to_run;

  if (steps_to_run < steps_to_decel) {
    motor->trapezoidal = 1;
    motor->run_step = (uint16_t)steps_to_run;
    motor->decel_step = (uint16_t)steps_to_decel;
  } else {
    motor->decel_step = motor->total_steps / 2;
  }
}

void start_motion(MotorProfile_t *motor, TIM_TypeDef *TIMx,
                  uint32_t total_steps) {
  // Configure move
  motor->total_steps = total_steps;
  motor->step_count = 0;
  motor->accel_count = 1;
  motor->rest = 0;

  // Calculate profile
  calculate_motion_profile(motor);
  motor->step_delay = calculate_initial_delay(motor);

  // Start motor
  motor->state = MOTOR_STATE_ACCEL;
  TIMx->ARR = motor->step_delay;
  TIMx->CR1 |= TIM_CR1_CEN;
}

static void update_step_delay(MotorProfile_t *motor) {
  int32_t numerator = 2 * motor->step_delay + motor->rest;
  int32_t denominator = 4 * motor->accel_count + 1;

  uint16_t new_step_delay = motor->step_delay - (numerator / denominator);
  int32_t new_rest = numerator % denominator;

  if (new_step_delay < motor->min_delay) {
    new_step_delay = motor->min_delay;
  }

  motor->step_delay = new_step_delay;
  motor->rest = new_rest;
  motor->accel_count++;
}

void motor_process_step(MotorProfile_t *motor) {
  motor->step_count++;

  switch (motor->state) {
    case MOTOR_STATE_ACCEL:
      update_step_delay(motor);

      if (motor->trapezoidal && motor->step_count >= motor->run_step) {
        motor->state = MOTOR_STATE_RUN;
      } else if (!motor->trapezoidal &&
                 motor->step_count >= motor->decel_step) {
        motor->state = MOTOR_STATE_DECEL;
        motor->accel_count = -motor->accel_count;
      }
      break;

    case MOTOR_STATE_RUN:
      if (motor->step_count >= motor->decel_step) {
        motor->state = MOTOR_STATE_DECEL;
        motor->accel_count = -motor->accel_count;
      }
      break;

    case MOTOR_STATE_DECEL:
      update_step_delay(motor);

      if (motor->step_count >= motor->total_steps) {
        motor->state = MOTOR_STATE_STOP;
      }
      break;

    default:
      motor->state = MOTOR_STATE_STOP;
      break;
  }
}

void stepper_init(StepperProfile_t *nema, GPIO_TypeDef *gpio_step,
                  GPIO_TypeDef *gpio_dir, long unsigned int pin_set,
                  long unsigned int pin_reset, long unsigned int pin_dir) {
  nema->GPIO_STEP = gpio_step;
  nema->GPIO_DIR = gpio_dir;
  nema->pin_set_mask = pin_set;
  nema->pin_reset_mask = pin_reset;
  nema->pin_dir_mask = pin_dir;
}

void configure_stepper(StepperProfile_t *nema, int master, int slave) {
  nema->motor_ratio_master = master;
  nema->motor_ratio_slave = slave;
  nema->motor_accumulator = 0;
  nema->motor_step_count = 0;
}

void step_motor(StepperProfile_t *nema) {
  nema->motor_accumulator += nema->motor_ratio_slave;

  if (nema->motor_accumulator >= nema->motor_ratio_master) {
    nema->GPIO_STEP->BSRR = nema->pin_set_mask;
    delay_us(1);
    nema->GPIO_STEP->BSRR = nema->pin_reset_mask;

    nema->motor_step_count++;
    nema->motor_accumulator -= nema->motor_ratio_master;
  }
}
