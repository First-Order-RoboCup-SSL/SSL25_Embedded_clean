#ifndef __SUPPORT_H
#define __SUPPORT_H

#include "main.h"

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cePort;
    uint16_t cePin;
    GPIO_TypeDef *csPort;
    uint16_t csPin;
} NRF24_TypeDef;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;



static inline void nRF24_CE_L(const NRF24_TypeDef *nrf) {
    HAL_GPIO_WritePin(nrf->cePort, nrf->cePin, GPIO_PIN_RESET);
}

static inline void nRF24_CE_H(const NRF24_TypeDef *nrf) {
    HAL_GPIO_WritePin(nrf->cePort, nrf->cePin, GPIO_PIN_SET);
}

static inline void nRF24_CSN_L(const NRF24_TypeDef *nrf) {
    HAL_GPIO_WritePin(nrf->csPort, nrf->csPin, GPIO_PIN_RESET);
}

static inline void nRF24_CSN_H(const NRF24_TypeDef *nrf) {
    HAL_GPIO_WritePin(nrf->csPort, nrf->csPin, GPIO_PIN_SET);
}


static inline uint8_t nRF24_LL_RW(const NRF24_TypeDef *nrf, uint8_t data) {
    // Wait until TX buffer is empty
    uint8_t result;
    if(HAL_SPI_TransmitReceive(nrf->hspi,&data,&result,1,2000)!=HAL_OK) {
        Error_Handler();
    };
    return result;
}


static inline void Delay_ms(uint32_t ms) { HAL_Delay(ms); }

#endif //__SUPPORT_H