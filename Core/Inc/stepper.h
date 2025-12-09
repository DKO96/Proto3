#ifndef STEPPER_H
#define STEPPER_H

#include "Types/pin_types.h"
#include "Types/stepper_types.h"

/**
 * @brief Initialize stepper joint with GPIO configurations
 */
void stepper_init(volatile StepperJoint_t *joint,
                  const StepperPinConfig_t *pins);

/**
 * @brief Set direction of stepper joint
 */
void stepper_direction_set(volatile StepperJoint_t *joint, StepDirection_t dir);

/**
 * @brief Configure stepper for synchronized motion
 */
void stepper_configure_sync(volatile StepperJoint_t *joint, StepDirection_t dir,
                            int reference_steps, int joint_steps);

/**
 * @brief Execute stepper motor synchronization
 */
void stepper_sync_step(volatile StepperJoint_t *joint);

/**
 * @brief Get current joint angle
 */
float calculate_angle(const volatile StepperJoint_t *joint);

/**
 * @brief Get desired number of steps for joint
 */
int calculate_steps(const volatile StepperJoint_t *joint, float target_rad);

#endif /* STEPPER_H */
