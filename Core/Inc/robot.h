#ifndef ROBOT_H
#define ROBOT_H

#include "Types/robot_types.h"

/**
 * @brief Initialize robot pins and configs
 */
void robot_init(RobotHandle_t *robot,
                const StepperPinConfig_t pin_configs[ROBOT_NUM_JOINTS],
                float link1_mm, float link2_mm);

/**
 * @brief Perform inverse kinematics based on robot position and target location
 */
JointAngles_t robot_inverse_kinematics(const RobotHandle_t *robot,
                                       const CartesianPoint_t *target);

/**
 * @brief Calculate steps for each robot joint
 */
MotionPlan_t robot_plan_motion(const RobotHandle_t *robot,
                               const JointAngles_t *target);

/**
 * @brief Sync stepper motors and start robot motion
 */
void robot_execute_plan(RobotHandle_t *robot, const MotionPlan_t *plan,
                        TIM_TypeDef *TIMx);

/**
 * @brief Monitor robot safety status, motion type, and robot state
 */
void robot_monitor(RobotHandle_t *robot);

#endif /* ROBOT_H */
