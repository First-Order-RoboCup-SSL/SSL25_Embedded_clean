#include "bsp_inverse_kinematics.h"
#include <stdint.h>
#include <math.h>

// Define M_PI if not defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

// Define the global debug structures
kinematics_debug_t kinematics_debug = {0};
velocity_debug_t velocity_debug = {0};

void BSP_InverseKinematics_Init(void)
{
    // Initialize debug structures
    kinematics_debug.update_count = 0;
    velocity_debug.update_count = 0;
    for(int i = 0; i < 4; i++) {
        velocity_debug.target_wheel_speeds[i] = 0;
        velocity_debug.target_motor_speeds[i] = 0;
        velocity_debug.actual_motor_speeds[i] = 0;
    }
    for(int i = 0; i < 3; i++) {
        kinematics_debug.raw_inputs[i] = 0;
    }
    kinematics_debug.vx = 0;
    kinematics_debug.vy = 0;
    kinematics_debug.omega = 0;
}

// Map remote control values to velocities
void BSP_MapRemoteToVelocities(int16_t x_input, int16_t y_input, int16_t r_input,
                              float* vx, float* vy, float* omega)
{
    // Store raw inputs
    kinematics_debug.raw_inputs[0] = r_input;
    kinematics_debug.raw_inputs[1] = y_input;
    kinematics_debug.raw_inputs[2] = x_input;
    
    // Map inputs to velocities with quadratic scaling for better control
    float x_norm = -(float)(x_input - REMOTE_MID) / REMOTE_MID;  // Invert X direction
    float y_norm = -(float)(y_input - REMOTE_MID) / REMOTE_MID;  // Invert Y direction
    float r_norm = (float)(r_input - REMOTE_MID) / REMOTE_MID;   // Keep rotation direction
    
    // Apply quadratic scaling while preserving sign
    *vx = (x_norm >= 0 ? 1 : -1) * x_norm * x_norm * MAX_LINEAR_VEL;
    *vy = (y_norm >= 0 ? 1 : -1) * y_norm * y_norm * MAX_LINEAR_VEL;
    *omega = (r_norm >= 0 ? 1 : -1) * r_norm * r_norm * MAX_ANGULAR_VEL;
    
    // Store calculated velocities for debugging
    kinematics_debug.vx = *vx;
    kinematics_debug.vy = *vy;
    kinematics_debug.omega = *omega;
}

// Calculate wheel velocities from robot velocities
void BSP_InverseKinematics_Calculate(float vx, float vy, float omega, float* wheel_velocities)
{
    // Store wheel angles for easier access
    float wheel_angles[4] = {
        WHEEL1_ANGLE,
        WHEEL2_ANGLE,
        WHEEL3_ANGLE,
        WHEEL4_ANGLE
    };
    
    // Calculate wheel velocities using inverse kinematics
    for(int i = 0; i < 4; i++) {
        float theta = wheel_angles[i];
        // Project robot velocity onto wheel direction and add rotational component
        wheel_velocities[i] = -vx * sinf(theta) + 
                             vy * cosf(theta) + 
                             omega * ROBOT_RADIUS;
        
        // Convert linear velocity to angular velocity (rad/s)
        wheel_velocities[i] /= WHEEL_RADIUS;
        
        // Store target wheel speeds for debugging
        velocity_debug.target_wheel_speeds[i] = wheel_velocities[i];
    }
    
    // Find maximum wheel speed for normalization
    float max_speed = 0;
    for(int i = 0; i < 4; i++) {
        float abs_speed = fabsf(wheel_velocities[i]);
        if(abs_speed > max_speed) max_speed = abs_speed;
    }
    
    // Normalize wheel speeds if they exceed the maximum
    if(max_speed > MAX_WHEEL_SPEED) {
        float scale = MAX_WHEEL_SPEED / max_speed;
        for(int i = 0; i < 4; i++) {
            wheel_velocities[i] *= scale;
        }
    }
    
    // Convert wheel speeds to motor speeds through gear ratio
    for(int i = 0; i < 4; i++) {
        velocity_debug.target_motor_speeds[i] = wheel_velocities[i] * GEAR_RATIO;
    }
    
    velocity_debug.update_count++;
}

// Convert wheel velocities to motor currents for DM3519
void BSP_ConvertVelocityToCurrent(float* wheel_velocities, int16_t* motor_currents)
{
    // Simple proportional control for current
    const float KP = 0.5f; // Proportional gain, adjust as needed
    
    for(int i = 0; i < 4; i++) {
        // Calculate current based on desired velocity
        float current = wheel_velocities[i] * KP;
        
        // Limit current to maximum
        if(current > MAX_CURRENT) current = MAX_CURRENT;
        if(current < -MAX_CURRENT) current = -MAX_CURRENT;
        
        // Convert to DM3519 current scale (-16384 to 16384)
        motor_currents[i] = (int16_t)(current * CURRENT_SCALE / MAX_CURRENT);
    }
} 