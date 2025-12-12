#ifndef CONFIG_H
#define CONFIG_H

#define M_PI 3.14159265358979323846

/* Robot geometry [mm] */
#define LINK_1 52.0f
#define LINK_2 100.0f
#define ROBOT_NUM_LINKS 2
#define ROBOT_NUM_JOINTS 3

/* Motor configuration */
#define STEPS_PER_REV 3200.0f
#define ALPHA (2.0f * M_PI / STEPS_PER_REV)  // [radians per step]
#define INITIAL_DELAY_CONST 0.676f

/* Motion profile */
#define MAX_SPEED 26     // [radians per second]
#define ACCELERATION 75  // [radians per second^2]
#define MIN_DELAY 75     // [microseconds]

/* Timer */
#define TIMER_FREQ 1000000

/* RTOS */
#define MONITOR_PRIORITY 1
#define WAYPOINT_PRIORITY 1
#define INV_KIN_PRIORITY 2
#define MOTOR_PRIORITY 3

#define WAYPOINT_QUEUE 2
#define INV_KIN_QUEUE 1

#endif /* CONFIG_H */