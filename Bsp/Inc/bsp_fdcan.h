#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H

#include "fdcan.h"

/* Message IDs for DM3519 motors */
#define FDCAN_MSG_ID_CONTROL     0x200  // Control frame ID for motors 1-4
#define FDCAN_MSG_ID_CONTROL_2   0x1FF  // Control frame ID for motors 5-8
#define FDCAN_MSG_ID_FEEDBACK_1  0x201  // Motor 1 feedback ID
#define FDCAN_MSG_ID_FEEDBACK_2  0x202  // Motor 2 feedback ID
#define FDCAN_MSG_ID_FEEDBACK_3  0x203  // Motor 3 feedback ID
#define FDCAN_MSG_ID_FEEDBACK_4  0x204  // Motor 4 feedback ID
#define FDCAN_MSG_ID_ERROR_CLEAR 0x7FF  // Error clear command ID

/* Function declarations */
void BSP_FDCAN_Init(void);
void BSP_FDCAN_Filter_Init(void);
HAL_StatusTypeDef BSP_FDCAN_Send_Message(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint32_t len);
HAL_StatusTypeDef BSP_FDCAN_Receive_Message(FDCAN_HandleTypeDef *hfdcan, uint8_t *data);
HAL_StatusTypeDef BSP_FDCAN_Send_Motor_Command(FDCAN_HandleTypeDef *hfdcan, int16_t *motor_currents);
HAL_StatusTypeDef BSP_FDCAN_Clear_Error(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);

void BSP_FDCAN1_RxCallback(void);
void BSP_FDCAN2_RxCallback(void);
void BSP_FDCAN3_RxCallback(void);

#endif /* BSP_FDCAN_H */ 