#include "bsp_inverse_kinematics.h"
#include <stdint.h>
#include <math.h>

// Define M_PI if not defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Define the global debug structures
kinematics_debug_t kinematics_debug = {0};
velocity_debug_t velocity_debug = {0};

void BSP_InverseKinematics_Init(void)
{
    // Initialize debug structures
    kinematics_debug.update_count = 0;
    velocity_debug.update_count = 0;
    for(int i = 0; i < 3; i++) {
        kinematics_debug.raw_inputs[i] = 0;
        velocity_debug.target_wheel_speeds[i] = 0;
        velocity_debug.target_motor_speeds[i] = 0;
        velocity_debug.actual_motor_speeds[i] = 0;
    }
    kinematics_debug.vx = 0;
    kinematics_debug.vy = 0;
    kinematics_debug.omega = 0;
}

// Map remote control values to velocities
void BSP_MapRemoteToVelocities(int16_t forward_back, int16_t left_right, int16_t rotation,
                              float* vx, float* vy, float* omega)
{
    // Store raw inputs
    kinematics_debug.raw_inputs[0] = rotation;
    kinematics_debug.raw_inputs[1] = forward_back;
    kinematics_debug.raw_inputs[2] = left_right;
    
    // Map forward/back to vy (forward is positive)
    float vy_norm = -(float)(forward_back - REMOTE_MID) / REMOTE_MID;  // Forward is positive
    *vy = vy_norm * MAX_LINEAR_VEL;
    
    // Map left/right to vx (right is positive)
    float vx_norm = -(float)(left_right - REMOTE_MID) / REMOTE_MID;  // Right is positive
    *vx = vx_norm * MAX_LINEAR_VEL;
    
    // Map rotation to omega (clockwise is positive)
    float omega_norm = (float)(rotation - REMOTE_MID) / REMOTE_MID;  // Clockwise is positive
    *omega = omega_norm * MAX_ANGULAR_VEL;
    
    // Store calculated velocities
    kinematics_debug.vx = *vx;
    kinematics_debug.vy = *vy;
    kinematics_debug.omega = *omega;
}

// Calculate motor currents from velocities
void BSP_InverseKinematics_Calculate(float vx, float vy, float omega, float* wheel_velocities)
{
    const float L = ROBOT_RADIUS;    // Distance from robot center to wheel center
    const float r = WHEEL_RADIUS;    // Wheel radius
    
    // Calculate wheel velocities (rad/s at the wheel) using the inverse kinematics
    // For each wheel, we project the desired robot velocity onto the wheel's force vector
    // and add the rotational component. The negative sign is because we want to achieve
    // the opposite of the wheel's natural force vector direction when needed.
    
    // Motor ID1 (index 0) - Front-right wheel (-15бу)
    // When rotating clockwise: force vector = (-0.258, 0.966)
    wheel_velocities[0] = -(vx * WHEEL1_VX + vy * WHEEL1_VY) / r + L * omega / r;
                      
    // Motor ID2 (index 1) - Rear wheel (90бу)
    // When rotating clockwise: force vector = (1, 0)
    wheel_velocities[1] = -(vx * WHEEL2_VX + vy * WHEEL2_VY) / r + L * omega / r;
                      
    // Motor ID3 (index 2) - Front-left wheel (-165бу)
    // When rotating clockwise: force vector = (-0.258, -0.966)
    wheel_velocities[2] = -(vx * WHEEL3_VX + vy * WHEEL3_VY) / r + L * omega / r;
    
    // Store target wheel speeds for debugging
    for(int i = 0; i < 3; i++) {
        velocity_debug.target_wheel_speeds[i] = wheel_velocities[i];
    }
    
    // Find maximum wheel speed for normalization
    float max_speed = 0;
    for(int i = 0; i < 3; i++) {
        float abs_speed = fabsf(wheel_velocities[i]);
        if(abs_speed > max_speed) max_speed = abs_speed;
    }
    
    // Calculate max allowed wheel angular speed (rad/s)
    float max_wheel_angular_speed = MAX_LINEAR_VEL / WHEEL_RADIUS;
    // Normalize wheel speeds if they exceed the maximum
    if(max_speed > max_wheel_angular_speed) {
        float scale = max_wheel_angular_speed / max_speed;
        for(int i = 0; i < 3; i++) {
            wheel_velocities[i] *= scale;
        }
    }
    
    // Convert wheel speeds to motor speeds by multiplying by gear ratio
    for(int i = 0; i < 3; i++) {
        velocity_debug.target_motor_speeds[i] = wheel_velocities[i] * GEAR_RATIO;
    }
    
    // Increment update counter
    velocity_debug.update_count++;
} 