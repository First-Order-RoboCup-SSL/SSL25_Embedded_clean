#ifndef BSP_INVERSE_KINEMATICS_H
#define BSP_INVERSE_KINEMATICS_H

#include <stdint.h>

// Robot physical parameters (in meters)
#define ROBOT_RADIUS 0.061f    // 61mm - Distance from robot center to wheel center
#define WHEEL_RADIUS 0.027f    // 27mm - Wheel radius
#define GEAR_RATIO 10.0f      // 10:1 gear reduction ratio

// Wheel force vector angles when rotating clockwise (in radians)
// These angles represent the direction of force vector produced by clockwise wheel rotation
// Measured counterclockwise from +y axis
// Motor ID1 (index 0) - Front-right wheel
#define WHEEL1_ANGLE (-15.0f * M_PI / 180.0f)  // -15° force vector when clockwise
// Motor ID2 (index 1) - Rear wheel
#define WHEEL2_ANGLE (90.0f * M_PI / 180.0f)   // 90° force vector when clockwise
// Motor ID3 (index 2) - Front-left wheel
#define WHEEL3_ANGLE (-165.0f * M_PI / 180.0f) // -165° force vector when clockwise

// Force vector components for each wheel when rotating clockwise
#define WHEEL1_VX -0.258f  // cos(-15°)
#define WHEEL1_VY 0.966f   // sin(-15°)
#define WHEEL2_VX 1.0f     // cos(90°)
#define WHEEL2_VY 0.0f     // sin(90°)
#define WHEEL3_VX -0.258f  // cos(-165°)
#define WHEEL3_VY -0.966f  // sin(-165°)

// Motor parameters
#define MAX_CURRENT 0.6f  // Maximum current in Amperes

// Velocity limits
#define MAX_LINEAR_VEL 1.5f    // Maximum linear velocity in m/s (increased to 1.5)
#define MAX_ANGULAR_VEL 6.0f    // Maximum angular velocity in rad/s (unchanged)

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
    float target_wheel_speeds[3];    // Target wheel speeds from inverse kinematics (rad/s)
    float target_motor_speeds[3];    // Target motor speeds after gear ratio (rad/s)
    float actual_motor_speeds[3];    // Actual motor speeds from encoder feedback (rad/s)
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

#endif // BSP_INVERSE_KINEMATICS_H 