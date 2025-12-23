#include "motion.h"

#include <math.h>

#include "config.h"
#include "main.h"

static uint16_t calculate_initial_delay(volatile MotionProfile_t *profile) {
  float initial_step = INITIAL_DELAY_CONST * TIMER_FREQ *
                       sqrtf(2.0f * ALPHA / profile->accel_rate);
  return (uint16_t)initial_step;
}

static void calculate_profile_shape(volatile MotionProfile_t *profile) {
  float steps_to_max_speed = (profile->max_speed * profile->max_speed) /
                             (2.0f * ALPHA * profile->accel_rate);

  uint32_t accel_steps = (uint32_t)steps_to_max_speed;

  if (2 * accel_steps < profile->total_steps) {
    /* Trapezoidal profile - max speed reached */
    profile->profile_type = MOTION_PROFILE_TRAPEZOIDAL;
    profile->accel_end_step = accel_steps;
    profile->decel_start_step = profile->total_steps - accel_steps;
  } else {
    /* Triangular profile - never reach max speed */
    profile->profile_type = MOTION_PROFILE_TRIANGULAR;
    profile->accel_end_step = profile->total_steps / 2;
    profile->decel_start_step = profile->total_steps - profile->accel_end_step;
  }
}

static void calculate_step_delay(volatile MotionProfile_t *profile) {
  int32_t numerator = 2 * profile->step_delay + profile->delay_remainder;
  int32_t denominator = 4 * profile->accel_count + 1;

  int32_t delta = numerator / denominator;
  int32_t remainder = numerator % denominator;

  uint16_t new_delay = profile->step_delay - delta;

  /* Clamp delay to min_delay*/
  if (new_delay < profile->min_delay) {
    new_delay = profile->min_delay;
  }

  profile->step_delay = (uint16_t)new_delay;
  profile->delay_remainder = remainder;
  profile->accel_count++;
}

void motion_configure(volatile MotionProfile_t *profile, float max_speed,
                      uint16_t accel_rate, uint16_t min_delay) {
  profile->max_speed = max_speed;
  profile->accel_rate = accel_rate;
  profile->min_delay = min_delay;
  profile->state = MOTION_STATE_IDLE;
}

void motion_start(volatile MotionProfile_t *profile, TIM_TypeDef *TIMx,
                  uint32_t total_steps) {
  /* Initialize motion state */
  profile->total_steps = total_steps;
  profile->current_step = 0;
  profile->accel_count = 1;
  profile->delay_remainder = 0;

  /* Calculate profile shape*/
  calculate_profile_shape(profile);

  /* Calculate initial_delay*/
  profile->step_delay = calculate_initial_delay(profile);

  /* Update profile state */
  profile->state = MOTION_STATE_ACCEL;

  /* Start timer and motion */
  TIMx->ARR = profile->step_delay;
  TIMx->CR1 |= TIM_CR1_CEN;
}

void motion_stop(MotionProfile_t *profile, TIM_TypeDef *TIMx) {
  TIMx->CR1 &= ~TIM_CR1_CEN;
  profile->state = MOTION_STATE_IDLE;
}

void motion_process_step(volatile MotionProfile_t *profile) {
  profile->current_step++;

  switch (profile->state) {
    case MOTION_STATE_ACCEL:
      calculate_step_delay(profile);

      if (profile->profile_type == MOTION_PROFILE_TRAPEZOIDAL) {
        /* Trapezoidal : transition to cruise when acceleration complete */
        if (profile->current_step >= profile->accel_end_step) {
          profile->state = MOTION_STATE_CONST;
        }
      } else {
        /* Triangular : transition to deceleration when acceleration complete */
        if (profile->current_step >= profile->decel_start_step) {
          profile->state = MOTION_STATE_DECEL;
          profile->accel_count = -profile->accel_count;
        }
      }
      break;

    case MOTION_STATE_CONST:
      if (profile->current_step >= profile->decel_start_step) {
        profile->state = MOTION_STATE_DECEL;
        profile->accel_count = -profile->accel_count;
      }
      break;

    case MOTION_STATE_DECEL:
      calculate_step_delay(profile);

      if (profile->current_step >= profile->total_steps) {
        profile->state = MOTION_STATE_IDLE;
      }
      break;

    case MOTION_STATE_IDLE:
    default:
      /* Should not reach here during normal operation */
      profile->state = MOTION_STATE_IDLE;
      break;
  }
}