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

// Define the global debug structure
velocity_debug_t velocity_debug = {0};

void BSP_InverseKinematics_Init(void)
{
    // Initialize debug structure
    velocity_debug.update_count = 0;
    for(int i = 0; i < 4; i++) {
        velocity_debug.target_wheel_speeds[i] = 0;
        velocity_debug.target_motor_speeds[i] = 0;
        velocity_debug.actual_motor_speeds[i] = 0;
    }
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
        wheel_velocities[i] = vy * sinf(theta) + 
                             -vx * cosf(theta) + 
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