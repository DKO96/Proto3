#ifndef CONFIG_H
#define CONFIG_H

#define M_PI 3.14159265358979323846

/* Robot geometry [mm] */
#define LINK_1 85.0f
#define LINK_2 100.0f
#define ROBOT_NUM_LINKS 2
#define ROBOT_NUM_JOINTS 3

/* Motor configuration */
#define STEPS_PER_REV 3200.0f
#define ALPHA (2.0f * M_PI / STEPS_PER_REV)  // [radians]

/* Motion profile */
#define INITIAL_DELAY_CONST 0.676f
#define MAX_SPEED 8.0f   // [radians per second]
#define ACCELERATION 12  // [radians per second^2]
#define MIN_DELAY 90     // [microseconds]

// Slow test
// #define MAX_SPEED 3.0f  // [radians per second]
// #define ACCELERATION 6  // [radians per second^2]
// #define MIN_DELAY 90    // [microseconds]

/* Timer */
#define TIMER_FREQ 1000000

/* RTOS */
#define MONITOR_PRIORITY 1
#define WAYPOINT_PRIORITY 1
#define INV_KIN_PRIORITY 2
#define MOTOR_PRIORITY 3
#define SAFETY_PRIORITY 4

#define WAYPOINT_QUEUE 1
#define INV_KIN_QUEUE 1

#endif /* CONFIG_H */