#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H

#include "fdcan.h"

/* Message IDs */
#define FDCAN_MSG_ID_FEEDBACK_1 0x201  // Motor 1 feedback ID
#define FDCAN_MSG_ID_FEEDBACK_2 0x202  // Motor 2 feedback ID
#define FDCAN_MSG_ID_FEEDBACK_3 0x203  // Motor 3 feedback ID

/* Function declarations */
void BSP_FDCAN_Init(void);
void BSP_FDCAN_Filter_Init(void);
HAL_StatusTypeDef BSP_FDCAN_Send_Message(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint32_t len);
HAL_StatusTypeDef BSP_FDCAN_Receive_Message(FDCAN_HandleTypeDef *hfdcan, uint8_t *data);

void BSP_FDCAN1_RxCallback(void);
void BSP_FDCAN2_RxCallback(void);
void BSP_FDCAN3_RxCallback(void);

#endif /* BSP_FDCAN_H */ 