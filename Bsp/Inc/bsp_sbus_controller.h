#ifndef BSP_SBUS_CONTROLLER_H
#define BSP_SBUS_CONTROLLER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// SBUS Protocol Constants
#define SBUS_START_BYTE      0x0F
#define SBUS_END_BYTE        0x00
#define SBUS_END_BYTE_ALT    0x04  // Alternative end byte for failsafe
#define SBUS_FRAME_LENGTH    25
#define SBUS_NUM_CHANNELS    4     // We only need 4 channels

// Signal timeout parameters
#define SBUS_TIMEOUT_MS      500   // Consider signal lost after 500ms
#define SBUS_FAILSAFE_VALUE  2048  // Center position (for safety)

// Channel value ranges
#define SBUS_MIN_VALUE       240   // Observed minimum SBUS value
#define SBUS_MAX_VALUE       1807  // Observed maximum SBUS value
#define MAPPED_MIN_VALUE     0     // Target minimum value
#define MAPPED_MAX_VALUE     4095  // Target maximum value (12-bit)

// Make sure these arrays are visible in debug
#if defined ( __CC_ARM )
#define DEBUG_VISIBLE __attribute__((used))
#elif defined ( __ICCARM__ )
#define DEBUG_VISIBLE __root
#elif defined ( __GNUC__ )
#define DEBUG_VISIBLE __attribute__((used))
#else
#define DEBUG_VISIBLE
#endif

// SBUS channel data arrays - marked for debug visibility
extern DEBUG_VISIBLE volatile int16_t sbus_channels[SBUS_NUM_CHANNELS];        // Raw SBUS values
extern DEBUG_VISIBLE volatile int16_t sbus_channels_mapped[SBUS_NUM_CHANNELS]; // Mapped values (0-4095)
extern DEBUG_VISIBLE volatile uint32_t sbus_last_update;                       // Timestamp of last valid frame
extern DEBUG_VISIBLE volatile bool sbus_signal_lost;                           // Signal loss indicator

// Initialize SBUS receiver
void BSP_SBUS_Init(void);

// Process received SBUS data byte
void BSP_SBUS_ProcessData(uint8_t byte);

// Get pointer to raw channel data array
int16_t* BSP_SBUS_GetChannels(void);

// Get pointer to mapped channel data array (0-4095)
int16_t* BSP_SBUS_GetMappedChannels(void);

// Check if SBUS signal is valid
bool BSP_SBUS_IsSignalValid(void);

// Start UART reception for SBUS
void BSP_SBUS_UART_StartReceive(void);

// UART reception complete callback (to be called from HAL callback)
void BSP_SBUS_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif // BSP_SBUS_CONTROLLER_H 