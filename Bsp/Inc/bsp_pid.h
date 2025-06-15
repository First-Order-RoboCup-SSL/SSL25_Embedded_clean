#ifndef BSP_PID_H
#define BSP_PID_H

#include <stdint.h>

/* Volatile PID parameters for easy debugging and tuning */
extern volatile float g_pid_kp;  // Proportional gain
extern volatile float g_pid_ki;  // Integral gain
extern volatile float g_pid_kd;  // Derivative gain

/* PID output limits */
#define PID_OUTPUT_MAX  6000.0f  // 6A max current (0.6A * 10000)
#define PID_OUTPUT_MIN -6000.0f  // -6A min current (-0.6A * 10000)

/* Integral limits to prevent windup */
#define PID_INTEGRAL_MAX  1000.0f
#define PID_INTEGRAL_MIN -1000.0f

/* Deadband parameters */
extern volatile float g_velocity_deadband;  // Velocity error deadband (rad/s)
extern volatile float g_current_deadband;   // Current output deadband

/* Control loop timing */
extern volatile uint8_t g_control_period_ms;  // Control loop period in milliseconds

/* Velocity control parameters */
#define VELOCITY_FILTER   0.2f    // Low-pass filter coefficient (0-1)

typedef struct {
    volatile float kp;
    volatile float ki;
    volatile float kd;
    float error_sum;
    float last_error;
} pid_param_t;

extern pid_param_t pid_params[3];

/* Function declarations */
float BSP_PID_Calculate(float target, float actual, float* error_sum, float* last_error);
float BSP_PID_Calculate_Indiv(pid_param_t* param, float target, float actual);

#endif /* BSP_PID_H */ 