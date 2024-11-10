#ifndef _LORAMESHER_DATA_PACKET_H
#define _LORAMESHER_DATA_PACKET_H

#include "RouteDataPacket.h"

#include "BuildOptions.h"
#include "debug_root.h"

#pragma pack(1)
class DataPacket final: public RouteDataPacket {
public:
    uint8_t payload[];

    /**
     * @brief Delete function for Packets
     *
     * @param p Packet to be deleted
     */
    void operator delete(void* p) {
        ESP_LOGV(LM_TAG, "Deleting Data packet");
        DebugHeapOnFreeCheckAll((void*)p);
        vPortFree(p);
    }
};
#pragma pack()

#endif