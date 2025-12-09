#ifndef MOTION_H
#define MOTION_H

#include "Types/motion_types.h"

/**
 * @brief Configure motion profile parameters
 */
void motion_configure(volatile MotionProfile_t *profile, float max_speed,
                      uint16_t accel_rate, uint16_t min_delay);

/**
 * @brief Start a motion with the given number of steps
 */
void motion_start(volatile MotionProfile_t *profile, TIM_TypeDef *TIMx,
                  uint32_t total_steps);

/**
 * @brief Stop motion
 */
void motion_stop(MotionProfile_t *profile, TIM_TypeDef *TIMx);

/**
 * @brief Process one step (call from timer ISR)
 */
void motion_process_step(volatile MotionProfile_t *profile);

#endif /* MOTION_H */