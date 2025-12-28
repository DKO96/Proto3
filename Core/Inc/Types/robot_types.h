#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include "Types/motion_types.h"
#include "Types/pin_types.h"
#include "Types/stepper_types.h"
#include "config.h"

typedef enum {
  SAFETY_STATE_OFF,
  SAFETY_STATE_ON,
} SafetyState_t;

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
  volatile SafetyState_t safety;
} RobotHandle_t;

#define LED_ON() (GPIOA->ODR |= GPIO_ODR_OD5)
#define LED_OFF() (GPIOA->ODR &= ~GPIO_ODR_OD5)

#endif /* ROBOT_TYPES_H */