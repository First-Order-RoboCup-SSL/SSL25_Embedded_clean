#include "bsp_fdcan.h"
#include "bsp_motor_feedback.h"
#include <string.h>

/* Private variables */
static uint8_t rx_buffer[8];  // 8 bytes buffer for receiving data

/**
  * @brief  Initialize FDCAN peripherals
  * @param  None
  * @retval None
  */
void BSP_FDCAN_Init(void)
{
    /* Initialize FDCAN filters */
    BSP_FDCAN_Filter_Init();
    
    /* Initialize motor feedback system */
    BSP_MotorFeedback_Init();
    
    /* Start FDCAN peripherals */
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_Start(&hfdcan3);
    
    /* Activate notification for new messages */
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/**
  * @brief  Initialize FDCAN filters
  * @param  None
  * @retval None
  */
void BSP_FDCAN_Filter_Init(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
    
    /* Configure filter for motor feedback messages */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x200;  // Base ID for motor messages
    sFilterConfig.FilterID2 = 0x7F0;  // Mask to accept IDs 0x201-0x203
    
    /* Configure filters for all FDCAN instances */
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
    HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);
    HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig);
    
    /* Configure global filters to reject non-matching frames */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
}

/**
  * @brief  Send message through FDCAN
  * @param  hfdcan: FDCAN handle
  * @param  id: Message ID
  * @param  data: Pointer to data buffer
  * @param  len: Length of data (max 8 bytes)
  * @retval HAL status
  */
HAL_StatusTypeDef BSP_FDCAN_Send_Message(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef tx_header;
    
    /* Configure header */
    tx_header.Identifier = id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;
    
    /* Add message to FIFO */
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, data);
}

/**
  * @brief  Process received feedback message
  * @param  rx_header: Pointer to CAN message header
  * @param  data: Pointer to received data
  * @retval None
  */
static void ProcessFeedbackMessage(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *data)
{
    // Get motor ID from CAN ID (0x201 -> 1, 0x202 -> 2, 0x203 -> 3)
    uint8_t motor_id = rx_header->Identifier - 0x200;
    
    // Process through motor feedback system if it's a valid motor ID
    if(motor_id >= 1 && motor_id <= 3) {
        BSP_MotorFeedback_ProcessMessage(motor_id, data);
    }
}

/**
  * @brief  Receive message from FDCAN
  * @param  hfdcan: FDCAN handle
  * @param  data: Pointer to data buffer
  * @retval HAL status
  */
HAL_StatusTypeDef BSP_FDCAN_Receive_Message(FDCAN_HandleTypeDef *hfdcan, uint8_t *data)
{
    FDCAN_RxHeaderTypeDef rx_header;
    
    /* Get message from FIFO */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, data) == HAL_OK)
    {
        ProcessFeedbackMessage(&rx_header, data);
        return HAL_OK;
    }
    
    return HAL_ERROR;
}

/**
  * @brief  FDCAN1 Rx callback
  * @param  None
  * @retval None
  */
void BSP_FDCAN1_RxCallback(void)
{
    BSP_FDCAN_Receive_Message(&hfdcan1, rx_buffer);
}

/**
  * @brief  FDCAN2 Rx callback
  * @param  None
  * @retval None
  */
void BSP_FDCAN2_RxCallback(void)
{
    BSP_FDCAN_Receive_Message(&hfdcan2, rx_buffer);
}

/**
  * @brief  FDCAN3 Rx callback
  * @param  None
  * @retval None
  */
void BSP_FDCAN3_RxCallback(void)
{
    BSP_FDCAN_Receive_Message(&hfdcan3, rx_buffer);
}

/**
  * @brief  FDCAN Rx FIFO 0 callback
  * @param  hfdcan: FDCAN handle
  * @param  RxFifo0ITs: indicates which interrupts are signalled
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    // Check if new message received
    if(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    {
        BSP_FDCAN_Receive_Message(hfdcan, rx_buffer);
    }
} 