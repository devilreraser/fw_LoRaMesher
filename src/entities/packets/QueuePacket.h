#ifndef _LORAMESHER_QUEUE_PACKET_H
#define _LORAMESHER_QUEUE_PACKET_H

#include "BuildOptions.h"

template <typename T>
class QueuePacket {
public:
    uint16_t number = 0;
    uint8_t priority = 0;
    uint8_t resend = 0;
    TickType_t timeout = xTaskGetTickCount() + pdMS_TO_TICKS(MAX_RESEND_TIMEOUT_MS);
    float rssi = 0;
    float snr = 0;
    T* packet;
};

#endif
