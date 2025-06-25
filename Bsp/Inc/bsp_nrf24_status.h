/**
  ******************************************************************************
  * @file    bsp_nrf24_status.h
  * @brief   NRF24 Status Monitoring Module
  *          Provides status monitoring and buffer management for NRF24
  ******************************************************************************
  */

#ifndef __BSP_NRF24_STATUS_H
#define __BSP_NRF24_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "radio/nrf24.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
/**
  * @brief  NRF24 Communication Status Structure
  */
typedef struct {
    bool initialized;           // NRF24 module initialized successfully
    bool communication_ok;      // Communication with NRF24 is working
    bool tx_success;           // Last transmission was successful
    bool tx_failed;            // Last transmission failed
    uint32_t tx_count;         // Total successful transmissions
    uint32_t tx_error_count;   // Total failed transmissions
    uint32_t last_tx_time;     // Last transmission timestamp
    uint32_t last_rx_time;     // Last reception timestamp
    uint8_t last_status;       // Last read status register
    uint8_t config;            // Current CONFIG register value
    uint8_t rf_ch;            // Current RF channel
    uint8_t rf_setup;         // Current RF_SETUP register value
    uint8_t observe_tx;       // Current OBSERVE_TX register value
    uint8_t fifo_status;      // Current FIFO_STATUS register value
} NRF24_Status_t;

/* Exported variables --------------------------------------------------------*/
extern NRF24_Status_t nrf24_status;

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Initialize NRF24 status monitoring
  * @param  nrf: Pointer to NRF24 instance
  * @retval None
  */
void BSP_NRF24_Status_Init(const NRF24_TypeDef *nrf);

/**
  * @brief  Update NRF24 status (call periodically)
  * @param  nrf: Pointer to NRF24 instance
  * @retval None
  */
void BSP_NRF24_Status_Update(const NRF24_TypeDef *nrf);

/**
  * @brief  Get current NRF24 status
  * @retval NRF24_Status_t: Current status structure
  */
NRF24_Status_t BSP_NRF24_Status_Get(void);

/**
  * @brief  Reset status counters
  * @retval None
  */
void BSP_NRF24_Status_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_NRF24_STATUS_H */ 