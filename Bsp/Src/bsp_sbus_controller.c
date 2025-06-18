#include "bsp_sbus_controller.h"
#include <string.h>

// Global channel data arrays with debug visibility
#if defined(__GNUC__)
__attribute__((section(".data")))
__attribute__((used))
#endif
volatile int16_t sbus_channels[SBUS_NUM_CHANNELS] = {0};

#if defined(__GNUC__)
__attribute__((section(".data")))
__attribute__((used))
#endif
volatile int16_t sbus_channels_mapped[SBUS_NUM_CHANNELS] = {0};

// Signal status variables
volatile uint32_t sbus_last_update = 0;
volatile bool sbus_signal_lost = true;

// Static variables for internal state
static volatile bool frame_complete = false;
static volatile int16_t frame_index = 0;
static volatile uint8_t sbus_frame[SBUS_FRAME_LENGTH] = {0};
static volatile uint8_t sbus_rx_byte = 0;

// Linear mapping function (internal use)
static inline int16_t map_value(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max) {
    // Handle edge cases to prevent overflow
    if (x <= in_min) return out_min;
    if (x >= in_max) return out_max;
    
    // Perform linear mapping
    int32_t temp = (int32_t)(x - in_min) * (out_max - out_min);
    temp = temp / (in_max - in_min);
    return (int16_t)(temp + out_min);
}

void BSP_SBUS_Init(void)
{
    frame_index = 0;
    frame_complete = false;
    sbus_signal_lost = true;
    sbus_last_update = HAL_GetTick();
    memset((void*)sbus_channels, 0, sizeof(sbus_channels));
    memset((void*)sbus_channels_mapped, 0, sizeof(sbus_channels_mapped));
    memset((void*)sbus_frame, 0, sizeof(sbus_frame));
}

bool BSP_SBUS_IsSignalValid(void)
{
    // Check if we've received a valid frame recently
    uint32_t current_time = HAL_GetTick();
    if (current_time - sbus_last_update > SBUS_TIMEOUT_MS) {
        sbus_signal_lost = true;
        // Set all channels to failsafe value
        for (int i = 0; i < SBUS_NUM_CHANNELS; i++) {
            sbus_channels[i] = SBUS_FAILSAFE_VALUE;
            sbus_channels_mapped[i] = MAPPED_MAX_VALUE / 2; // Center position
        }
    }
    return !sbus_signal_lost;
}

void BSP_SBUS_ProcessData(uint8_t byte)
{
    // If the first byte is not SBUS start byte, discard and restart
    if (frame_index == 0 && byte != SBUS_START_BYTE) {
        return;
    }
    

    sbus_frame[frame_index++] = byte;

    // Consider a frame complete when full frame is received
    if (frame_index == SBUS_FRAME_LENGTH) {
        frame_complete = true;
        frame_index = 0;

        // Verify the end byte
        if (sbus_frame[24] == SBUS_END_BYTE || sbus_frame[24] == SBUS_END_BYTE_ALT) {
            // Update signal status
            sbus_last_update = HAL_GetTick();
            sbus_signal_lost = false;

            // Parse channel data according to SBUS protocol
            // Each channel is 11 bits, packed across bytes
            sbus_channels[0] = ((sbus_frame[1]      | sbus_frame[2] << 8)                           & 0x07FF);
            sbus_channels[1] = ((sbus_frame[2] >> 3 | sbus_frame[3] << 5)                           & 0x07FF);
            sbus_channels[2] = ((sbus_frame[3] >> 6 | sbus_frame[4] << 2 | sbus_frame[5] << 10)    & 0x07FF);
            sbus_channels[3] = ((sbus_frame[5] >> 1 | sbus_frame[6] << 7)                           & 0x07FF);

            // Map each channel value to 0-4095 range
            for (int i = 0; i < SBUS_NUM_CHANNELS; i++) {
                if (sbus_channels[i] < SBUS_MIN_VALUE) sbus_channels[i] = SBUS_MIN_VALUE;
                if (sbus_channels[i] > SBUS_MAX_VALUE) sbus_channels[i] = SBUS_MAX_VALUE;
                sbus_channels_mapped[i] = map_value(sbus_channels[i], 
                                                  SBUS_MIN_VALUE, 
                                                  SBUS_MAX_VALUE, 
                                                  MAPPED_MIN_VALUE, 
                                                  MAPPED_MAX_VALUE);
            }
        }
    }
}

int16_t* BSP_SBUS_GetChannels(void)
{
    return (int16_t*)sbus_channels;
}

int16_t* BSP_SBUS_GetMappedChannels(void)
{
    return (int16_t*)sbus_channels_mapped;
}

void BSP_SBUS_UART_StartReceive(void)
{
    // Start UART5 interrupt reception (1 byte at a time)
    extern UART_HandleTypeDef huart5;
    HAL_UART_Receive_IT(&huart5, (uint8_t*)&sbus_rx_byte, 1);
}

void BSP_SBUS_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5) {
        // Process the received byte
        BSP_SBUS_ProcessData(sbus_rx_byte);
        
        // Continue receiving the next byte
        HAL_UART_Receive_IT(huart, (uint8_t*)&sbus_rx_byte, 1);
    }
} 