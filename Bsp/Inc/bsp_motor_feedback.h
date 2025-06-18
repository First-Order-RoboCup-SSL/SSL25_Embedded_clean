#ifndef BSP_MOTOR_FEEDBACK_H
#define BSP_MOTOR_FEEDBACK_H

#include <stdint.h>

// Motor feedback data structure for DM3519
typedef struct {
    int16_t rotor_angle;        // 0~8191, mapping to 0~360бу
    int16_t rotor_speed;        // RPM
    int16_t torque_current;     // -16384~16384, mapping to -20.5~20.5A
    uint8_t temperature;        // буC
    uint8_t error_status;       // Error status byte
} motor_feedback_t;

// Motor feedback buffer for all 4 motors
typedef struct {
    motor_feedback_t motors[4];  // Index corresponds to motor ID-1
    uint32_t last_update_time;   // Last update timestamp
    uint32_t update_count;       // Number of updates received
} motor_feedback_buffer_t;

// Function declarations
void BSP_MotorFeedback_Init(void);
void BSP_MotorFeedback_ProcessMessage(uint32_t motor_id, uint8_t* data);
motor_feedback_t* BSP_MotorFeedback_GetMotorData(uint8_t motor_id);
uint32_t BSP_MotorFeedback_GetLastUpdateTime(void);
uint32_t BSP_MotorFeedback_GetUpdateCount(void);
uint8_t BSP_MotorFeedback_GetErrorStatus(uint8_t motor_id);

#endif // BSP_MOTOR_FEEDBACK_H 