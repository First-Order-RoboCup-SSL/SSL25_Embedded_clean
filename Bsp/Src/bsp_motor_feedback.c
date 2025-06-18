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
    for(int i = 0; i < 4; i++) {
        feedback_buffer.motors[i].rotor_angle = 0;
        feedback_buffer.motors[i].rotor_speed = 0;
        feedback_buffer.motors[i].torque_current = 0;
        feedback_buffer.motors[i].temperature = 0;
        feedback_buffer.motors[i].error_status = 0;
        velocity_debug.actual_motor_speeds[i] = 0;
    }
    feedback_buffer.last_update_time = 0;
    feedback_buffer.update_count = 0;
}

void BSP_MotorFeedback_ProcessMessage(uint32_t motor_id, uint8_t* data)
{
    // Check if motor_id is valid (1-4)
    if(motor_id < 1 || motor_id > 4 || data == NULL) {
        return;
    }

    // Calculate array index (0-3) from motor ID (1-4)
    uint8_t index = motor_id - 1;

    // Parse the data according to DM3519 protocol
    // Rotor angle (0~8191)
    feedback_buffer.motors[index].rotor_angle = (int16_t)((data[0] << 8) | data[1]);
    
    // Rotor speed (RPM)
    feedback_buffer.motors[index].rotor_speed = (int16_t)((data[2] << 8) | data[3]);
    
    // Torque current (-16384~16384 -> -20.5~20.5A)
    feedback_buffer.motors[index].torque_current = (int16_t)((data[4] << 8) | data[5]);
    
    // Temperature (¡ãC)
    feedback_buffer.motors[index].temperature = data[6];
    
    // Error status
    feedback_buffer.motors[index].error_status = data[7];

    // Store rotor speed in rad/s for debugging
    velocity_debug.actual_motor_speeds[index] = feedback_buffer.motors[index].rotor_speed * RPM_TO_RAD_PER_SEC;

    // Update timing information
    feedback_buffer.last_update_time = HAL_GetTick();
    feedback_buffer.update_count++;
}

motor_feedback_t* BSP_MotorFeedback_GetMotorData(uint8_t motor_index)
{
    // Check if motor_index is valid (0-3)
    if(motor_index >= 4) {
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

uint8_t BSP_MotorFeedback_GetErrorStatus(uint8_t motor_id)
{
    // Check if motor_id is valid (1-4)
    if(motor_id < 1 || motor_id > 4) {
        return 0xFF;  // Invalid motor ID
    }
    return feedback_buffer.motors[motor_id - 1].error_status;
} 