#include "stepper.h"

#include "config.h"
#include "main.h"

void stepper_init(volatile StepperJoint_t *joint,
                  const StepperPinConfig_t *pins) {
  /* Configure step pin*/
  joint->step_port = pins->step.port;
  joint->step_set = GPIO_PIN_SET_MASK(pins->step.pin);
  joint->step_reset = GPIO_PIN_RESET_MASK(pins->step.pin);

  /* Configure direction pin*/
  joint->dir_port = pins->dir.port;
  joint->dir_set = GPIO_PIN_SET_MASK(pins->dir.pin);
  joint->dir_reset = GPIO_PIN_RESET_MASK(pins->dir.pin);

  /* Initialize synchronization */
  joint->sync_denominator = 0;
  joint->sync_numerator = 0;
  joint->sync_accumulator = 0;

  /* Initialize position */
  joint->position_steps = (int32_t)(pins->init_rad / ALPHA);
  joint->direction = STEP_DIR_POSITIVE;
}

void stepper_direction_set(volatile StepperJoint_t *joint,
                           StepDirection_t dir) {
  joint->direction = dir;

  if (dir == STEP_DIR_POSITIVE) {
    joint->dir_port->BSRR = joint->dir_set;
  } else {
    joint->dir_port->BSRR = joint->dir_reset;
  }
}

void stepper_configure_sync(volatile StepperJoint_t *joint, StepDirection_t dir,
                            int reference_steps, int joint_steps) {
  stepper_direction_set(joint, dir);

  joint->sync_denominator = reference_steps;
  joint->sync_numerator = joint_steps;
  joint->sync_accumulator = 0;
}

void stepper_sync_step(volatile StepperJoint_t *joint) {
  joint->sync_accumulator += joint->sync_numerator;

  if (joint->sync_accumulator >= joint->sync_denominator) {
    /* Generate step pulse */
    joint->step_port->BSRR = joint->step_set;
    delay_us(1);
    joint->step_port->BSRR = joint->step_reset;

    /* Update accumulator */
    joint->sync_accumulator -= joint->sync_denominator;

    /* Update position */
    if (joint->direction) {
      joint->position_steps++;
    } else {
      joint->position_steps--;
    }
  }
}

float calculate_angle(const volatile StepperJoint_t *joint) {
  return (float)joint->position_steps * ALPHA;
}

int calculate_steps(const volatile StepperJoint_t *joint, float target_rad) {
  float current_angle = calculate_angle(joint);
  float delta_angle = target_rad - current_angle;

  return (int)(delta_angle / ALPHA);
}