#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include "Types/motion_types.h"
#include "Types/pin_types.h"
#include "Types/stepper_types.h"
#include "config.h"

typedef struct {
  float x;
  float y;
  float z;
} CartesianPoint_t;

typedef struct {
  float theta[3];
  uint8_t valid;
} JointAngles_t;

typedef struct {
  int joint_steps[ROBOT_NUM_JOINTS];
  int reference_steps;
  uint8_t valid;
} MotionPlan_t;

typedef struct {
  volatile StepperJoint_t joints[ROBOT_NUM_JOINTS];
  volatile MotionProfile_t motion;
  float link_lengths[ROBOT_NUM_LINKS];
} RobotHandle_t;

#endif /* ROBOT_TYPES_H */