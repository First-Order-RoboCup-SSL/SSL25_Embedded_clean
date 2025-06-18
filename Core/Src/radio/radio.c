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

typedef struct {
  uint16_t adc_ch1;          // ADC Channel 1 (2 bytes)
  uint16_t adc_ch2;          // ADC Channel 2 (2 bytes)  
  uint16_t adc_ch3;          // ADC Channel 3 (2 bytes)
  uint16_t adc_ch4;          // ADC Channel 4 (2 bytes)
  uint8_t aux1_state;        // AUX1 button state (1 byte)
  uint8_t aux2_state;        // AUX2 button state (1 byte)
} __attribute__((packed)) NRF24_TxBuffer_t;

extern uint16_t decoded[6];

void poll(const NRF24_TypeDef *nrf) {
    
    if (nRF24_GetStatus_RXFIFO(nrf) != nRF24_STATUS_RXFIFO_EMPTY) {
        uint8_t payload[10];
        uint8_t length = 10;

        uint8_t pipe = nRF24_ReadPayload(nrf, payload, &length);
        uint8_t ackPayload = !HAL_GPIO_ReadPin(IR_SENSOR_GPIO_Port, IR_SENSOR_Pin);
        nRF24_WriteAckPayload(nrf, pipe, &ackPayload, 1);
        // SEGGER_RTT_printf(0, "RX Pipe: %d, Status: %d\n", pipe, nRF24_GetStatus_RXFIFO(nrf));


        // // Compute CRC for the first 7 bytes of the payload
        // uint8_t computed_crc = compute_crc(payload, 7);
        // // Compare computed CRC with the received CRC (8th byte)
        // // if (computed_crc != payload[7]) {
        // //     SEGGER_RTT_WriteString(0, "CRC error: Invalid packet. Discarding packet.\n");
        // //     return;  // Discard the packet if CRC does not match
        // // }

        NRF24_TxBuffer_t data;
        memcpy(&data, &payload, 10);
        decoded[0] = data.adc_ch1;
        decoded[1] = data.adc_ch2;
        decoded[2] = data.adc_ch3;
        decoded[4] = data.adc_ch4;
    }

    // Flush TX FIFO and clear IRQ flags
    nRF24_ClearIRQFlags(nrf);
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
    nRF24_SetRXPipe(nrf, nRF24_PIPE1, nRF24_AA_ON, 10); // Auto-ACK: enabled, payload length: 8 bytes
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

    uint8_t *ptr = (uint8_t *)nRF24_TEST_ADDR;
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

void parsePacket(uint8_t *bytes, Packet* packet) {
    // Parse the 16-bit half precision floats from the first 6 bytes
    packet->forward = half_to_float((((uint16_t) bytes[0]) << 8) + bytes[1]);
    packet->left = half_to_float((((uint16_t) bytes[2]) << 8) + bytes[3]);
    packet->angular = half_to_float((((uint16_t) bytes[4]) << 8) + bytes[5]);

    // Parse control flags from the 7th byte
    uint8_t flags = bytes[6];
    packet->kicker_bottom = (flags & 0x80) != 0;
    packet->kicker_top = (flags & 0x40) != 0;
    packet->dribble = (flags & 0x20) != 0;
}
