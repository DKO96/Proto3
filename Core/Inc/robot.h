#ifndef ROBOT_H
#define ROBOT_H

#include "Types/robot_types.h"

void robot_init(RobotHandle_t *robot,
                const StepperPinConfig_t pin_configs[ROBOT_NUM_JOINTS],
                float link1_mm, float link2_mm);

JointAngles_t robot_inverse_kinematics(const RobotHandle_t *robot,
                                       const CartesianPoint_t *target);

MotionPlan_t robot_plan_motion(const RobotHandle_t *robot,
                               const JointAngles_t *target);

void robot_execute_plan(RobotHandle_t *robot, const MotionPlan_t *plan,
                        TIM_TypeDef *TIMx);

#endif /* ROBOT_H */
