#include "bsp_pid.h"
#include "main.h"
#include <math.h>

// Define global deadband parameters with initial values
volatile deadband_params_t g_deadband_params = {
    .velocity_deadband = 0.2f,           // 0.2 rad/s
    .current_deadband = 164.0f           // 30mA (164 = 30mA * 16384/3000mA)
};

// Define PID parameters for 4 motors
pid_param_t pid_params[4] = {
    {50.0f, 1.0f, 6.0f, 0.0f, 0.0f, 0.0f}, // motor1
    {50.0f, 1.0f, 6.0f, 0.0f, 0.0f, 0.0f}, // motor2
    {50.0f, 1.0f, 6.0f, 0.0f, 0.0f, 0.0f}, // motor3
    {50.0f, 1.0f, 6.0f, 0.0f, 0.0f, 0.0f}  // motor4
};

void BSP_PID_Init(void)
{
    for(int i = 0; i < 4; i++) {
        BSP_PID_Reset(i);
    }
}

void BSP_PID_Reset(uint8_t motor_index)
{
    if(motor_index >= 4) return;
    
    pid_params[motor_index].error_sum = 0.0f;
    pid_params[motor_index].last_error = 0.0f;
    pid_params[motor_index].last_output = 0.0f;
}

void BSP_PID_UpdateDeadbandParams(float velocity_db, float current_db)
{
    g_deadband_params.velocity_deadband = velocity_db;
    g_deadband_params.current_deadband = current_db;
}

float BSP_PID_Calculate(uint8_t motor_index, float target, float actual)
{
    if(motor_index >= 4) return 0.0f;
    
    pid_param_t* param = &pid_params[motor_index];
    float error = target - actual;
    
    // Apply velocity deadband
    if (fabsf(error) < g_deadband_params.velocity_deadband) {
        error = 0;
        param->error_sum = 0;  // Reset integral when in deadband
        param->last_output = 0;
        return 0;
    }
    
    // Calculate PID terms
    float dt = CONTROL_PERIOD_MS / 1000.0f;  // Convert ms to seconds
    
    // P term
    float p_term = param->kp * error;
    
    // I term with anti-windup
    param->error_sum += error * dt;
    if (param->error_sum > PID_INTEGRAL_MAX) param->error_sum = PID_INTEGRAL_MAX;
    else if (param->error_sum < PID_INTEGRAL_MIN) param->error_sum = PID_INTEGRAL_MIN;
    float i_term = param->ki * param->error_sum;
    
    // D term with filtering
    float d_term = param->kd * (error - param->last_error) / dt;
    param->last_error = error;
    
    // Calculate base output
    float output = p_term + i_term + d_term;
    
    // Apply current deadband with hysteresis
    if (fabsf(output) < g_deadband_params.current_deadband) {
        if (fabsf(param->last_output) < g_deadband_params.current_deadband) {
            output = 0;
        }
    } else {
        // Add or subtract deadband based on direction
        output += (output > 0) ? g_deadband_params.current_deadband : -g_deadband_params.current_deadband;
    }
    
    // Apply output limits
    if (output > PID_OUTPUT_MAX) output = PID_OUTPUT_MAX;
    else if (output < PID_OUTPUT_MIN) output = PID_OUTPUT_MIN;
    
    // Apply output filtering
    output = output * (1.0f - VELOCITY_FILTER) + param->last_output * VELOCITY_FILTER;
    param->last_output = output;
    
    return output;
} 