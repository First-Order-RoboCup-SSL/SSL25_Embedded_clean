#ifndef BSP_MOTOR_FEEDBACK_H
#define BSP_MOTOR_FEEDBACK_H

#include <stdint.h>

// Motor feedback data structure
typedef struct {
    int16_t mechanical_angle;    // 0~8191, mapping to 0~360??
    int16_t rotor_speed;        // RPM
    int16_t actual_torque;      // -10000~10000, mapping to -10~10A
} motor_feedback_t;

// Motor feedback buffer for all 3 motors
typedef struct {
    motor_feedback_t motors[3];  // Index corresponds to motor ID-1
    uint32_t last_update_time;   // Last update timestamp
    uint32_t update_count;       // Number of updates received
} motor_feedback_buffer_t;

// Function declarations
void BSP_MotorFeedback_Init(void);
void BSP_MotorFeedback_ProcessMessage(uint32_t motor_id, uint8_t* data);
motor_feedback_t* BSP_MotorFeedback_GetMotorData(uint8_t motor_id);
uint32_t BSP_MotorFeedback_GetLastUpdateTime(void);
uint32_t BSP_MotorFeedback_GetUpdateCount(void);

#endif // BSP_MOTOR_FEEDBACK_H 