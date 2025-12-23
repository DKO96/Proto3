#include "robot.h"

#include <math.h>
#include <stdlib.h>

#include "motion.h"
#include "stepper.h"
#include "uart.h"

void robot_init(RobotHandle_t *robot,
                const StepperPinConfig_t pin_configs[ROBOT_NUM_JOINTS],
                float link1_mm, float link2_mm) {
  /* Store link lengths */
  robot->link_lengths[0] = link1_mm;
  robot->link_lengths[1] = link2_mm;

  /* Initialize joints */
  for (uint8_t i = 0; i < ROBOT_NUM_JOINTS; i++) {
    stepper_init(&robot->joints[i], &pin_configs[i]);
  }

  /* Initialize motion profile */
  motion_configure(&robot->motion, MAX_SPEED, ACCELERATION, MIN_DELAY);
}

JointAngles_t robot_inverse_kinematics(const RobotHandle_t *robot,
                                       const CartesianPoint_t *target) {
  JointAngles_t result = {.valid = 0};

  float x = target->x;
  float y = target->y;
  float z = target->z;

  float L1 = robot->link_lengths[0];
  float L2 = robot->link_lengths[1];

  float r = sqrtf(x * x + y * y);

  float cos_theta2 = (r * r + z * z - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);

  if (fabsf(cos_theta2) > 1) return result;

  result.theta[2] = acosf(cos_theta2);

  result.theta[1] = atan2f(r, z) - atan2f(L2 * sinf(result.theta[2]),
                                          L1 + L2 * cosf(result.theta[2]));

  result.theta[0] = atan2f(y, x);

  result.valid = 1;

  return result;
}

MotionPlan_t robot_plan_motion(const RobotHandle_t *robot,
                               const JointAngles_t *target) {
  MotionPlan_t plan = {.valid = 0, .reference_steps = 0};

  if (!target->valid) {
    return plan;
  }

  for (uint8_t i = 0; i < ROBOT_NUM_JOINTS; i++) {
    plan.joint_steps[i] = calculate_steps(&robot->joints[i], target->theta[i]);

    /* Track maximum absolute steps for reference axis */
    int abs_steps = abs(plan.joint_steps[i]);
    if (abs_steps > plan.reference_steps) {
      plan.reference_steps = abs_steps;
    }
  }

  plan.valid = (plan.reference_steps > 0);

  return plan;
}

void robot_execute_plan(RobotHandle_t *robot, const MotionPlan_t *plan,
                        TIM_TypeDef *TIMx) {
  if (!plan->valid) {
    return;
  }

  for (uint8_t i = 0; i < ROBOT_NUM_JOINTS; i++) {
    StepDirection_t dir =
        (plan->joint_steps[i] >= 0) ? STEP_DIR_POSITIVE : STEP_DIR_NEGATIVE;

    int joint_steps = abs(plan->joint_steps[i]);

    stepper_configure_sync(&robot->joints[i], dir, plan->reference_steps,
                           joint_steps);
  }

  motion_start(&robot->motion, TIMx, plan->reference_steps);
}

void robot_monitor(RobotHandle_t *robot) {
  printS("\r\n=== Robot State ===\r\n");

  printS("Motion:        ");
  switch (robot->motion.profile_type) {
    case MOTION_PROFILE_TRAPEZOIDAL:
      printS("Trapezoidal");
      break;

    case MOTION_PROFILE_TRIANGULAR:
      printS("Triangular");
      break;

    default:
      break;
  }
  printS("\r\n");

  printS("State:         ");
  switch (robot->motion.state) {
    case MOTION_STATE_IDLE:
      printS("IDLE");
      break;

    case MOTION_STATE_ACCEL:
      printS("ACCEL");
      break;

    case MOTION_STATE_CONST:
      printS("CONST");
      break;

    case MOTION_STATE_DECEL:
      printS("DECEL");
      break;

    default:
      break;
  }
  printS("\r\n");

  printS("Joints:        ");
  printI(robot->joints[0].position_steps);
  printS("    ");
  printI(robot->joints[1].position_steps);
  printS("    ");
  printI(robot->joints[2].position_steps);
  printS("\r\n");
}