#ifndef BSP_INVERSE_KINEMATICS_H
#define BSP_INVERSE_KINEMATICS_H

#include <stdint.h>
#include <math.h>

// Robot physical parameters (in meters)
#define ROBOT_RADIUS 0.086f    // 86mm - Distance from robot center to wheel center
#define WHEEL_RADIUS 0.0262f   // 26.2mm - Wheel radius
#define GEAR_RATIO 4.0f       // 4:1 gear reduction ratio

// Wheel angles (in radians) - measured from positive x-axis
#define WHEEL1_ANGLE (M_PI_2 + 1.0f)        // Wheel 1
#define WHEEL2_ANGLE (3*M_PI_2 - 0.75f)     // Wheel 2
#define WHEEL3_ANGLE (3*M_PI_2 + 0.75f)     // Wheel 3
#define WHEEL4_ANGLE (M_PI_2 - 1.0f)        // Wheel 4

// Motor parameters for DM3519
#define MAX_CURRENT 3.0f       // Maximum current in Amperes (reduced from 20.5A to 3A)
#define CURRENT_SCALE 16384    // Current scale factor (-16384 to 16384 maps to -3A to 3A)

// Velocity limits
#define MAX_LINEAR_VEL 2.0f    // Maximum linear velocity in m/s (reduced from 5.0 to 2.0)
#define MAX_ANGULAR_VEL 4.0f   // Maximum angular velocity in rad/s (reduced from 5.0 to 4.0)
#define MAX_WHEEL_SPEED 60.0f  // Maximum wheel angular speed in rad/s

// Remote control parameters
#define REMOTE_MAX 4096
#define REMOTE_MID (REMOTE_MAX / 2)

// Debug structure
typedef struct {
    uint16_t raw_inputs[3];     // Raw remote control inputs
    float vx;                   // Calculated X velocity
    float vy;                   // Calculated Y velocity
    float omega;                // Calculated angular velocity
    uint32_t update_count;      // Number of updates performed
} kinematics_debug_t;

// Debug structure for velocity monitoring
typedef struct {
    float target_wheel_speeds[4];    // Target wheel speeds from inverse kinematics (rad/s)
    float target_motor_speeds[4];    // Target motor speeds after gear ratio (rad/s)
    float actual_motor_speeds[4];    // Actual motor speeds from encoder feedback (rad/s)
    uint32_t update_count;           // Number of updates performed
} velocity_debug_t;

// Global debug structures
extern kinematics_debug_t kinematics_debug;
extern velocity_debug_t velocity_debug;

// Function declarations
void BSP_InverseKinematics_Init(void);
void BSP_MapRemoteToVelocities(int16_t x_input, int16_t y_input, int16_t r_input,
                              float* vx, float* vy, float* omega);
void BSP_InverseKinematics_Calculate(float vx, float vy, float omega, float* wheel_velocities);
void BSP_ConvertVelocityToCurrent(float* wheel_velocities, int16_t* motor_currents);

#endif // BSP_INVERSE_KINEMATICS_H 