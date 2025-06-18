#ifndef BSP_PID_H
#define BSP_PID_H

#include <stdint.h>

/* PID output limits for DM3519 */
#define PID_OUTPUT_MAX  16384.0f  // Maps to 3A max current
#define PID_OUTPUT_MIN -16384.0f  // Maps to -3A min current

/* Integral limits to prevent windup */
#define PID_INTEGRAL_MAX  5000.0f
#define PID_INTEGRAL_MIN -5000.0f

/* Deadband parameters */
typedef struct {
    float velocity_deadband;     // Velocity error deadband (rad/s)
    float current_deadband;      // Current output deadband
} deadband_params_t;

extern volatile deadband_params_t g_deadband_params;

/* Control loop timing */
#define CONTROL_PERIOD_MS 1  // 1ms control loop (1kHz) to match DM3519 feedback rate

/* Velocity control parameters */
#define VELOCITY_FILTER   0.2f    // Low-pass filter coefficient (0-1)

/* PID parameters structure */
typedef struct {
    volatile float kp;           // Proportional gain
    volatile float ki;           // Integral gain
    volatile float kd;           // Derivative gain
    float error_sum;            // Integral term accumulator
    float last_error;           // Previous error for derivative
    float last_output;          // Previous output for filtering
} pid_param_t;

/* Motor-specific PID parameters */
extern pid_param_t pid_params[4];  // Now 4 motors

/* Function declarations */
void BSP_PID_Init(void);
float BSP_PID_Calculate(uint8_t motor_index, float target, float actual);
void BSP_PID_Reset(uint8_t motor_index);
void BSP_PID_UpdateDeadbandParams(float velocity_db, float current_db);

#endif /* BSP_PID_H */ 