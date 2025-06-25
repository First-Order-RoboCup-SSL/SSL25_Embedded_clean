#ifndef BSP_PID_H
#define BSP_PID_H

#include <stdint.h>

/* PID output limits for DM3519 */
#define PID_OUTPUT_MAX  16384.0f  // Maps to 3A max current
#define PID_OUTPUT_MIN -16384.0f  // Maps to -3A min current

/* Integral limits to prevent windup */
#define PID_INTEGRAL_MAX  5000.0f
#define PID_INTEGRAL_MIN -5000.0f

/* Control loop timing */
#define CONTROL_PERIOD_MS 1  // 1ms control loop (1kHz) to match DM3519 feedback rate

/* Debug-friendly parameters (watchable in debugger) */
extern volatile float g_velocity_filter;  // Low-pass filter coefficient (0-1)

/* Debug-friendly deadband parameters (watchable in debugger) */
typedef struct {
    volatile float velocity_deadband;     // Velocity error deadband (rad/s) - Adjust in debugger
    volatile float current_deadband;      // Current output deadband (scaled: value * 16384/3000 for mA)
} deadband_params_t;

/* Global PID parameters - Watch and modify these in debugger */
typedef struct {
    volatile float kp;      // Global proportional gain
    volatile float ki;      // Global integral gain
    volatile float kd;      // Global derivative gain
} pid_gains_t;

extern volatile pid_gains_t g_pid_gains;  // Global PID gains - adjust in debugger
extern volatile deadband_params_t g_deadband_params;  // Deadband parameters

/* Per-motor PID state structure */
typedef struct {
    float error_sum;            // Integral term accumulator
    float last_error;           // Previous error for derivative
    float last_output;          // Previous output for filtering
    volatile float output;      // Current output (for debugging)
} pid_state_t;

/* Motor-specific PID states */
extern pid_state_t pid_states[4];  // State for 4 motors

/* Function declarations */
void BSP_PID_Init(void);
float BSP_PID_Calculate(uint8_t motor_index, float target, float actual);
void BSP_PID_Reset(uint8_t motor_index);
void BSP_PID_UpdateDeadbandParams(float velocity_db, float current_db);
void BSP_PID_UpdateGains(float kp, float ki, float kd);

#endif /* BSP_PID_H */ 