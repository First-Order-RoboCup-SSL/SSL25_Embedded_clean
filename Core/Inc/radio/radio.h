//
// Created by sam on 23/01/2025.
//

#ifndef RADIO_H
#define RADIO_H

#include "main.h"
#include "support.h"
#include "nrf24.h"
#include <stdbool.h>

void radio(NRF24_TypeDef *nrf);

typedef struct {
    float forward;
    float left;
    float angular;
    bool kicker_bottom;
    bool kicker_top;
    bool dribble;
} Packet;

void poll(const NRF24_TypeDef *nrf);
void parsePacket(uint8_t *bytes, Packet *packet);

extern Packet currentPacket;

// volatile bool gNRFErrorDetected;

#endif //RADIO_H
