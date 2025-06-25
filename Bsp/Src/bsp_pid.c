#include "bsp_pid.h"
#include "main.h"
#include <math.h>

// Global debug parameters - Adjust these in debugger
volatile deadband_params_t g_deadband_params = {
    .velocity_deadband = 0.2f,           // 0.2 rad/s - Adjust in debugger
    .current_deadband = 164.0f           // ~30mA (164 = 30mA * 16384/3000mA) - Adjust in debugger
};

// Global PID gains - Adjust these in debugger
volatile pid_gains_t g_pid_gains = {
    .kp = 900.0f,  // Proportional gain
    .ki = 5.0f,    // Integral gain
    .kd = 3.0f     // Derivative gain
};

// Global filter coefficient - Adjust in debugger
volatile float g_velocity_filter = 0.2f;  // Default low-pass filter coefficient

// Define PID states for 4 motors
pid_state_t pid_states[4] = {
    {0.0f, 0.0f, 0.0f, 0.0f}, // motor1
    {0.0f, 0.0f, 0.0f, 0.0f}, // motor2
    {0.0f, 0.0f, 0.0f, 0.0f}, // motor3
    {0.0f, 0.0f, 0.0f, 0.0f}  // motor4
};

void BSP_PID_Init(void)
{
    // Initialize PID states
    for(int i = 0; i < 4; i++) {
        BSP_PID_Reset(i);
    }
}

void BSP_PID_Reset(uint8_t motor_index)
{
    if(motor_index >= 4) return;
    
    pid_states[motor_index].error_sum = 0.0f;
    pid_states[motor_index].last_error = 0.0f;
    pid_states[motor_index].last_output = 0.0f;
    pid_states[motor_index].output = 0.0f;
}

void BSP_PID_UpdateDeadbandParams(float velocity_db, float current_db)
{
    g_deadband_params.velocity_deadband = velocity_db;
    g_deadband_params.current_deadband = current_db;
}

void BSP_PID_UpdateGains(float kp, float ki, float kd)
{
    g_pid_gains.kp = kp;
    g_pid_gains.ki = ki;
    g_pid_gains.kd = kd;
}

float BSP_PID_Calculate(uint8_t motor_index, float target, float actual)
{
    if(motor_index >= 4) return 0.0f;
    
    pid_state_t* state = &pid_states[motor_index];
    float error = target - actual;
    
    // Apply velocity deadband
    if (fabsf(error) < g_deadband_params.velocity_deadband) {
        error = 0;
        state->error_sum = 0;  // Reset integral when in deadband
        state->last_output = 0;
        state->output = 0;
        return 0;
    }
    
    // Calculate PID terms
    float dt = CONTROL_PERIOD_MS / 1000.0f;  // Convert ms to seconds
    
    // P term
    float p_term = g_pid_gains.kp * error;
    
    // I term with anti-windup
    state->error_sum += error * dt;
    if (state->error_sum > PID_INTEGRAL_MAX) state->error_sum = PID_INTEGRAL_MAX;
    else if (state->error_sum < PID_INTEGRAL_MIN) state->error_sum = PID_INTEGRAL_MIN;
    float i_term = g_pid_gains.ki * state->error_sum;
    
    // D term with filtering
    float d_term = g_pid_gains.kd * (error - state->last_error) / dt;
    state->last_error = error;
    
    // Calculate base output
    float output = p_term + i_term + d_term;
    
    // Apply current deadband with hysteresis
    if (fabsf(output) < g_deadband_params.current_deadband) {
        if (fabsf(state->last_output) < g_deadband_params.current_deadband) {
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
    output = output * (1.0f - g_velocity_filter) + state->last_output * g_velocity_filter;
    state->last_output = output;
    
    // Store output for debugging
    state->output = output;
    
    return output;
} 