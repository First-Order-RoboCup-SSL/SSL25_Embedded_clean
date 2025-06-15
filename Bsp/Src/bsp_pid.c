#include "bsp_pid.h"
#include <math.h>

// Define global variables for deadband control
volatile float g_velocity_deadband = 0.2f;  // Initial value: 0.2 rad/s
volatile float g_current_deadband = 200.0f;  // Initial value: 200

// Define control loop timing
volatile uint8_t g_control_period_ms = 5;  // Initial value: 5ms (200Hz)

pid_param_t pid_params[3] = {
    {50.0f, 1.0f, 6.0f, 0.0f, 0.0f}, // motor1: Kp=50, Ki=1, Kd=6
    {50.0f, 1.0f, 6.0f, 0.0f, 0.0f}, // motor2: Kp=50, Ki=1, Kd=6
    {50.0f, 1.0f, 6.0f, 0.0f, 0.0f}  // motor3: Kp=50, Ki=1, Kd=6
};

float BSP_PID_Calculate_Indiv(pid_param_t* param, float target, float actual)
{
    float error = target - actual;
    
    // 1. Apply velocity deadband
    if (fabsf(error) < g_velocity_deadband) {
        error = 0;
        param->error_sum = 0;  // Reset integral when in deadband
        return 0;  // No output when in deadband
    }
    
    // 2. Calculate P term
    float p_term = param->kp * error;
    
    // 3. Calculate I term - consider control period
    float dt = g_control_period_ms / 1000.0f;  // Convert ms to seconds
    param->error_sum += error * dt;  // Integrate with respect to time
    if (param->error_sum > PID_INTEGRAL_MAX) param->error_sum = PID_INTEGRAL_MAX;
    else if (param->error_sum < PID_INTEGRAL_MIN) param->error_sum = PID_INTEGRAL_MIN;
    float i_term = param->ki * param->error_sum;
    
    // 4. Calculate D term - consider control period
    float d_term = param->kd * (error - param->last_error) / dt;  // Derivative with respect to time
    param->last_error = error;
    
    // 5. Calculate total output
    float output = p_term + i_term + d_term;
    
    // 6. Apply current deadband
    if (fabsf(output) < g_current_deadband) {
        output = 0;
    } else if (output > 0) {
        output = output + g_current_deadband;
    } else {
        output = output - g_current_deadband;
    }
    
    // 7. Final output limiting
    if (output > PID_OUTPUT_MAX) output = PID_OUTPUT_MAX;
    else if (output < PID_OUTPUT_MIN) output = PID_OUTPUT_MIN;
    
    return output;
}

float BSP_PID_Calculate(float target, float actual, float* error_sum, float* last_error)
{
    // Calculate error
    float error = target - actual;
    
    // Calculate P term
    float p_term = g_pid_kp * error;
    
    // Calculate I term
    *error_sum += error;
    
    // Apply integral limits (anti-windup)
    if (*error_sum > PID_INTEGRAL_MAX) {
        *error_sum = PID_INTEGRAL_MAX;
    } else if (*error_sum < PID_INTEGRAL_MIN) {
        *error_sum = PID_INTEGRAL_MIN;
    }
    float i_term = g_pid_ki * (*error_sum);
    
    // Calculate D term
    float d_term = g_pid_kd * (error - *last_error);
    *last_error = error;
    
    // Calculate total output
    float output = p_term + i_term + d_term;
    
    // Apply output limits
    if (output > PID_OUTPUT_MAX) {
        output = PID_OUTPUT_MAX;
    } else if (output < PID_OUTPUT_MIN) {
        output = PID_OUTPUT_MIN;
    }
    
    return output;
} 