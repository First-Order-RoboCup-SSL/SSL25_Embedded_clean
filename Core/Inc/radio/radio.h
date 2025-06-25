//
// Created by sam on 23/01/2025.
//

#ifndef RADIO_H
#define RADIO_H

#include "main.h"
#include "support.h"
#include "nrf24.h"
#include <stdbool.h>
#include <stdint.h>

void radio(NRF24_TypeDef *nrf);

// Raw packet structure matching the transmitter side
typedef struct {
    uint8_t robot_id;          // Robot ID
    uint16_t forward_vel;      // Forward velocity (float16)
    uint16_t left_vel;         // Left velocity (float16)
    uint16_t angular_vel;      // Angular velocity (float16)
    uint16_t dribbler;         // Dribbler state and speed
    uint8_t kickers;           // Upper and lower kicker power
} __attribute__((packed)) NRF24_RxBuffer_t;

// Processed packet with converted float values
typedef struct {
    uint8_t robot_id;          // Robot ID
    float forward_vel;         // Forward velocity (converted from float16)
    float left_vel;           // Left velocity (converted from float16)
    float angular_vel;        // Angular velocity (converted from float16)
    uint8_t dribbler_state;   // Dribbler state (2 bits)
    uint16_t dribbler_speed;  // Dribbler speed (14 bits)
    uint8_t upper_kicker;     // Upper kicker power (4 bits)
    uint8_t lower_kicker;     // Lower kicker power (4 bits)
} ProcessedPacket_t;

// Function declarations
void poll(const NRF24_TypeDef *nrf);
void parsePacket(uint8_t *bytes, ProcessedPacket_t *packet);
float float16_to_float32(uint16_t h);

// Global variables for debugging
extern ProcessedPacket_t currentPacket;
extern volatile NRF24_RxBuffer_t rawPacket;  // Raw buffer for debugging

// volatile bool gNRFErrorDetected;

#endif //RADIO_H
