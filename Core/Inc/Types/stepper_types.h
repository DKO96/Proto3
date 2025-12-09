#ifndef STEPPER_TYPES_H
#define STEPPER_TYPES_H

#include "stm32f446xx.h"

/**
 * @brief Step direction
 */
typedef enum {
  STEP_DIR_NEGATIVE,
  STEP_DIR_POSITIVE,
} StepDirection_t;

/**
 * @brief Stepper motor joint state
 *
 * This structure contains information required to control a single stepper
 * motor joint, including GPIO configuration, Bresenham synchronization
 * parameters, and position tracking.
 */
typedef struct {
  /* GPIO - STEP pin */
  GPIO_TypeDef *step_port;
  long unsigned int step_set;
  long unsigned int step_reset;

  /* GPIO - DIR pin */
  GPIO_TypeDef *dir_port;
  long unsigned int dir_set;
  long unsigned int dir_reset;

  /* Bresenham line algorithm synchronization */
  int sync_denominator;  // reference axis total steps
  int sync_numerator;    // joint's target steps
  int sync_accumulator;  // error accumulator

  /* Position tracking */
  int32_t position_steps;  // current position [steps]
  StepDirection_t direction;

} StepperJoint_t;

#endif /* STEPPER_TYPES_H */