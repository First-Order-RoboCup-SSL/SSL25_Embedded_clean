// 
// Created by sam on 23/01/2025.
// Updated by Han on 02/02/2025.
// 

#include "../../Inc/radio/radio.h"
#include "main.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>      // For uint8_t, uint16_t, etc.
#include "../../Inc/usdelay.h"
#include <string.h>

// Global variables
ProcessedPacket_t currentPacket;
volatile NRF24_RxBuffer_t rawPacket;  // Raw buffer for debugging

// Compute CRC-8 using polynomial 0x07 over the provided data buffer.
uint8_t compute_crc(const uint8_t* data, size_t length) {
    uint8_t crc = 0x00;
    uint8_t poly = 0x07;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (uint8_t)((crc << 1) ^ poly);
            else
                crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

void RTT_PrintFloatWithLabel(const char* label, float value) {
    // char buffer[32];
    // char* ptr = buffer;
    
    // // Copy label
    // while (*label) {
    //     *ptr++ = *label++;
    // }
    
    // // Add separator
    // *ptr++ = ':';
    // *ptr++ = ' ';
    
    // // Handle negative numbers
    // if (value < 0) {
    //     *ptr++ = '-';
    //     value = -value;
    // }
    
    // // Extract integer part
    // int32_t integer_part = (int32_t)value;
    
    // // Extract decimal part (fixed 3 decimal places)
    // float decimal = value - (float)integer_part;
    
    // // Convert integer to temporary buffer (reverse order)
    // char temp[16];
    // int pos = 0;
    // int32_t temp_int = integer_part;
    
    // // Handle zero case
    // if (temp_int == 0) {
    //     temp[pos++] = '0';
    // }
    
    // // Convert other numbers
    // while (temp_int > 0) {
    //     temp[pos++] = '0' + (temp_int % 10);
    //     temp_int /= 10;
    // }
    
    // // Copy to main buffer in correct order
    // while (pos > 0) {
    //     *ptr++ = temp[--pos];
    // }
    
    // // Add decimal point and 3 decimal places
    // *ptr++ = '.';
    // for (int i = 0; i < 3; i++) {
    //     decimal *= 10;
    //     int digit = (int)decimal;
    //     *ptr++ = '0' + digit;
    //     decimal -= digit;
    // }
    
    // // Add newline
    // *ptr++ = '\n';
    // *ptr = '\0';
    
    // // Print to RTT
    // SEGGER_RTT_WriteString(0, buffer);
}

void RTT_PrintBool(const char* label, bool value) {
    // Print label if provided
    // if (label) {
    //     // SEGGER_RTT_WriteString(0, label);
    //     // SEGGER_RTT_WriteString(0, ": ");
    // }
    
    // // Print boolean value
    // // SEGGER_RTT_WriteString(0, value ? "true" : "false");
    
    // // Add newline
    // SEGGER_RTT_WriteString(0, "\n");
}

float float16_to_float32(uint16_t h) {
    uint32_t sign = (h >> 15) & 0x1;
    uint32_t exponent = (h >> 10) & 0x1F;
    uint32_t mantissa = h & 0x3FF;

    uint32_t f32_sign = sign << 31;
    uint32_t f32_exponent;
    uint32_t f32_mantissa;

    if (exponent == 0) {
        if (mantissa == 0) {
            // Zero
            f32_exponent = 0;
            f32_mantissa = 0;
        } else {
            // Subnormal number
            // Normalize the mantissa
            int e = -1;
            uint32_t m = mantissa;
            while ((m & 0x400) == 0) { // While leading bit is not 1
                m <<= 1;
                e--;
            }
            m &= 0x3FF;  // remove leading 1

            f32_exponent = (127 - 15 + e) << 23; // Adjust exponent bias difference (127 vs 15)
            f32_mantissa = m << 13;
        }
    } else if (exponent == 0x1F) {
        // Inf or NaN
        f32_exponent = 0xFF << 23;
        f32_mantissa = mantissa ? (mantissa << 13) : 0;
    } else {
        // Normalized number
        f32_exponent = (exponent - 15 + 127) << 23;
        f32_mantissa = mantissa << 13;
    }

    uint32_t f32_bits = f32_sign | f32_exponent | f32_mantissa;

    union {
        uint32_t u;
        float f;
    } conv;

    conv.u = f32_bits;
    return conv.f;
}

void poll(const NRF24_TypeDef *nrf) {
    if (nRF24_GetStatus_RXFIFO(nrf) != nRF24_STATUS_RXFIFO_EMPTY) {
        uint8_t payload[10];  // 10 bytes for the complete packet
        uint8_t length = 10;

        uint8_t pipe = nRF24_ReadPayload(nrf, payload, &length);
        uint8_t ackPayload = !HAL_GPIO_ReadPin(IR_SENSOR_GPIO_Port, IR_SENSOR_Pin);
        nRF24_WriteAckPayload(nrf, pipe, &ackPayload, 1);

        // Copy raw data to debug buffer first
        memcpy((void*)&rawPacket, payload, sizeof(NRF24_RxBuffer_t));
        
        // Then parse into processed packet
        parsePacket(payload, &currentPacket);
    }

    // Flush TX FIFO and clear IRQ flags
    nRF24_ClearIRQFlags(nrf);
}

void parsePacket(uint8_t *bytes, ProcessedPacket_t* packet) {
    // Use the already copied raw packet
    packet->robot_id = rawPacket.robot_id;
    
    // Convert float16 velocities to float32
    packet->forward_vel = float16_to_float32(rawPacket.forward_vel);
    packet->left_vel = float16_to_float32(rawPacket.left_vel);
    packet->angular_vel = float16_to_float32(rawPacket.angular_vel);
    
    // Parse dribbler state and speed
    packet->dribbler_state = (rawPacket.dribbler >> 14) & 0x03;  // Top 2 bits
    packet->dribbler_speed = rawPacket.dribbler & 0x3FFF;        // Bottom 14 bits
    
    // Parse kicker values
    packet->upper_kicker = (rawPacket.kickers >> 4) & 0x0F;  // Upper 4 bits
    packet->lower_kicker = rawPacket.kickers & 0x0F;         // Lower 4 bits
}

void radio(NRF24_TypeDef *nrf) {
    // RX/TX disabled
    nRF24_CE_L(nrf);

    // Only test once, and go to error handler if failed
    uint8_t checks = 0;
    while (!nRF24_Check(nrf)) {
        if (checks > 10) {
            Error_Handler();
            return;
        }
        HAL_Delay(10);
        checks++;
    }


    // Initialize the nRF24L01 to its default state
    nRF24_Init(nrf);
    // This is simple receiver with Enhanced ShockBurst:
    //   - RX address: 'ESB'
    //   - payload: 10 bytes
    //   - RF channel: 40 (2440MHz)
    //   - data rate: 2Mbps
    //   - CRC scheme: 2 byte
    //
    // The transmitter sends a 10-byte packets to the address 'ESB' with Auto-ACK (ShockBurst enabled)

    nRF24_EnableAA(nrf, 0xFF);
    nRF24_SetRFChannel(nrf, 76);
    nRF24_SetDataRate(nrf, nRF24_DR_1Mbps);

    nRF24_SetCRCScheme(nrf, nRF24_CRC_2byte);

    nRF24_SetAddrWidth(nrf, 5);
    nRF24_SetTXPower(nrf, nRF24_TXPWR_12dBm);


    uint8_t nRF24_ADDR[] = "edoN1";
    nRF24_SetTxAddr(nrf, nRF24_ADDR, 5);
    nRF24_SetAddr(nrf, nRF24_PIPE1, nRF24_ADDR); // Program address for RX pipe #1
    nRF24_SetRXPipe(nrf, nRF24_PIPE1, nRF24_AA_ON, 10); // Auto-ACK: enabled, payload length: 10 bytes
    nRF24_ActivateFeatures(nrf);
    // Set operational mode (PRX == receiver)
    nRF24_SetOperationalMode(nrf, nRF24_MODE_RX);
    nRF24_SetDynamicPayloadLength(nrf, nRF24_DPL_ON);
    nRF24_SetPayloadWithAck(nrf, 1);

    HAL_Delay(10);

    // Wake the transceiver
    nRF24_SetPowerMode(nrf, nRF24_PWR_UP);
    

    nRF24_CE_L(nrf);
    HAL_Delay(10);
    nRF24_PrettyPrint(nrf);
    HAL_Delay(5000);

    // The main loop
    nRF24_CE_H(nrf);
}

float half_to_float(uint16_t half) {
    int sign = (half >> 15) & 0x1;
    int exponent = (half >> 10) & 0x1F;
    int fraction = half & 0x3FF;

    if (exponent == 0 && fraction == 0) {
        return sign ? -0.0f : 0.0f;
    }
    if (exponent == 0x1F) {
        return fraction ? NAN : (sign ? -INFINITY : INFINITY);
    }
    float result;
    if (exponent == 0) {
        result = ldexpf((float)fraction, -24);
    } else {
        result = ldexpf((float)(fraction | 0x400), exponent - 15 - 10);
    }
    return sign ? -result : result;
}
