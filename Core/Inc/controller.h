#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

#include "stm32f446xx.h"

typedef enum {
  STEPPER_0,
  STEPPER_1,
  STEPPER_2,
  STEPPER_3,
  STEPPER_4,
  STEPPER_5
} Stepper_t;

typedef struct {
  // Stepper info
  Stepper_t stepper;
  GPIO_TypeDef GPIOx;

  // Step Counter
  int motor_ratio_master;
  int motor_ratio_slave;
  int motor_accumulator;

  // Debug
  int motor_substeps;

} StepperProfile_t;

typedef enum {
  MOTOR_STATE_STOP,
  MOTOR_STATE_ACCEL,
  MOTOR_STATE_RUN,
  MOTOR_STATE_DECEL
} MotorState_t;

typedef struct {
  // Step timing parameters
  TIM_TypeDef TIMx;
  uint16_t step_delay;
  int32_t rest;
  int32_t accel_count;

  // Motion profile
  uint8_t trapezoidal;
  uint32_t total_steps;
  uint32_t step_count;
  uint16_t decel_step;
  uint16_t run_step;

  // Motor configuration
  float alpha;
  uint32_t timer_frequency;
  float max_speed;
  int16_t min_delay;
  int16_t accel_rate;

  // Motor state
  MotorState_t state;
} MotorProfile_t;

void master_init(MotorProfile_t *motor, float max_speed, uint16_t accel_rate,
                 uint16_t min_delay);
void start_motion(MotorProfile_t *motor, TIM_TypeDef *TIMx,
                  uint32_t total_steps);
void motor_process_step(MotorProfile_t *motor);

void stepper_init(StepperProfile_t *nema, Stepper_t motor, int master,
                  int slave);
void step_motor(StepperProfile_t *nema);

#endif /* MOTOR_CONTROL_H */