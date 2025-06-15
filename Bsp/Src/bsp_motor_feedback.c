#include "bsp_motor_feedback.h"
#include "bsp_inverse_kinematics.h"
#include "main.h"

// Static buffer for motor feedback data
static motor_feedback_buffer_t feedback_buffer = {0};

// Constants for speed conversion
#define RPM_TO_RAD_PER_SEC (2.0f * M_PI / 60.0f)  // Convert RPM to rad/s

void BSP_MotorFeedback_Init(void)
{
    // Initialize the feedback buffer
    for(int i = 0; i < 3; i++) {
        feedback_buffer.motors[i].mechanical_angle = 0;
        feedback_buffer.motors[i].rotor_speed = 0;
        feedback_buffer.motors[i].actual_torque = 0;
        velocity_debug.actual_motor_speeds[i] = 0;
    }
    feedback_buffer.last_update_time = 0;
    feedback_buffer.update_count = 0;
}

void BSP_MotorFeedback_ProcessMessage(uint32_t motor_id, uint8_t* data)
{
    // Check if motor_id is valid (1-3)
    if(motor_id < 1 || motor_id > 3 || data == NULL) {
        return;
    }

    // Calculate array index (0-2) from motor ID (1-3)
    uint8_t index = motor_id - 1;

    // Parse the data according to the protocol
    // Mechanical angle (0~8191)
    feedback_buffer.motors[index].mechanical_angle = (int16_t)((data[0] << 8) | data[1]);
    
    // Rotor speed (RPM)
    feedback_buffer.motors[index].rotor_speed = (int16_t)((data[2] << 8) | data[3]);
    
    // Actual torque current
    feedback_buffer.motors[index].actual_torque = (int16_t)((data[4] << 8) | data[5]);

    // Store rotor speed in rad/s for debugging
    // 注意：这里存储的是电机转子速度，所以不需要除以减速比
    velocity_debug.actual_motor_speeds[index] = feedback_buffer.motors[index].rotor_speed * RPM_TO_RAD_PER_SEC;

    // Update timing information
    feedback_buffer.last_update_time = HAL_GetTick();
    feedback_buffer.update_count++;
}

motor_feedback_t* BSP_MotorFeedback_GetMotorData(uint8_t motor_index)
{
    // Check if motor_index is valid (0-2)
    if(motor_index >= 3) {
        return NULL;
    }
    return &feedback_buffer.motors[motor_index];
}

uint32_t BSP_MotorFeedback_GetLastUpdateTime(void)
{
    return feedback_buffer.last_update_time;
}

uint32_t BSP_MotorFeedback_GetUpdateCount(void)
{
    return feedback_buffer.update_count;
} 