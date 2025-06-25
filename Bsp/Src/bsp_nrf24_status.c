/**
  ******************************************************************************
  * @file    bsp_nrf24_status.c
  * @brief   NRF24 Status Monitoring Module Implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_nrf24_status.h"

/* Private variables ---------------------------------------------------------*/
NRF24_Status_t nrf24_status = {0};

/* Functions -----------------------------------------------------------------*/

void BSP_NRF24_Status_Init(const NRF24_TypeDef *nrf) {
    // Reset all status fields
    nrf24_status.initialized = false;
    nrf24_status.communication_ok = false;
    nrf24_status.tx_success = false;
    nrf24_status.tx_failed = false;
    nrf24_status.tx_count = 0;
    nrf24_status.tx_error_count = 0;
    nrf24_status.last_tx_time = 0;
    nrf24_status.last_rx_time = 0;
    nrf24_status.last_status = 0;
    nrf24_status.config = 0;
    nrf24_status.rf_ch = 0;
    nrf24_status.rf_setup = 0;
    nrf24_status.observe_tx = 0;
    nrf24_status.fifo_status = 0;

    // Check if NRF24 is responding
    if (nRF24_Check(nrf)) {
        nrf24_status.initialized = true;
        nrf24_status.communication_ok = true;
    }
}

void BSP_NRF24_Status_Update(const NRF24_TypeDef *nrf) {
    // Update status registers
    nrf24_status.last_status = nRF24_GetStatus(nrf);
    nrf24_status.config = nRF24_GetConfig(nrf);
    nrf24_status.rf_ch = nRF24_GetRFChannel(nrf);
    nrf24_status.rf_setup = nRF24_ReadReg(nrf, nRF24_REG_RF_SETUP);
    nrf24_status.observe_tx = nRF24_GetRetransmitCounters(nrf);
    nrf24_status.fifo_status = nRF24_ReadReg(nrf, nRF24_REG_FIFO_STATUS);

    // Check for TX success/failure
    uint8_t status = nrf24_status.last_status;
    if (status & nRF24_FLAG_TX_DS) {
        nrf24_status.tx_success = true;
        nrf24_status.tx_failed = false;
        nrf24_status.tx_count++;
        nrf24_status.last_tx_time = HAL_GetTick();
    }
    if (status & nRF24_FLAG_MAX_RT) {
        nrf24_status.tx_success = false;
        nrf24_status.tx_failed = true;
        nrf24_status.tx_error_count++;
    }
    if (status & nRF24_FLAG_RX_DR) {
        nrf24_status.last_rx_time = HAL_GetTick();
    }

    // Clear interrupt flags
    nRF24_ClearIRQFlags(nrf);
}

NRF24_Status_t BSP_NRF24_Status_Get(void) {
    return nrf24_status;
}

void BSP_NRF24_Status_Reset(void) {
    nrf24_status.tx_count = 0;
    nrf24_status.tx_error_count = 0;
    nrf24_status.tx_success = false;
    nrf24_status.tx_failed = false;
} 