#ifndef MOTION_TYPES_H
#define MOTION_TYPES_H

#include "stm32f446xx.h"

/**
 * @brief Motion profile state machine states
 */
typedef enum {
  MOTION_STATE_IDLE,   // not moving
  MOTION_STATE_ACCEL,  // accelerating
  MOTION_STATE_CONST,  // constant velocity
  MOTION_STATE_DECEL   // decelerating
} MotionState_t;

/**
 * @brief Motion profile shape
 */
typedef enum {
  MOTION_PROFILE_TRIANGULAR,
  MOTION_PROFILE_TRAPEZOIDAL
} MotionProfileType_t;

/**
 * @brief Trapezoidal/triangular motion profile state
 *
 * Implements a stepper motor algorithm for smooth acceleration and deceleration
 * with linear ramping
 */
typedef struct {
  /* Step timing */
  uint16_t step_delay;      // current delay between steps [microseconds]
  int32_t delay_remainder;  // remainder for fixed-point math
  int32_t accel_count;      // step counter for delay calculation

  /* Motion profile shape */
  MotionProfileType_t profile_type;
  uint32_t total_steps;       // total steps for move
  uint32_t current_step;      // current step number
  uint32_t accel_end_step;    // step where acceleration ends
  uint32_t decel_start_step;  // steps where deceleration begins

  /* Configuration */
  float max_speed;      // maximum speed [radians per second]
  uint16_t min_delay;   // minimum timer delay [microseconds]
  uint16_t accel_rate;  // acceleration rate [radians per seconds^2]

  /* State machine */
  MotionState_t state;

} MotionProfile_t;

#endif /* MOTION_TYPES_H */