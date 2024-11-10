#ifndef _LORAMESHER_PACKET_FACTORY_H
#define _LORAMESHER_PACKET_FACTORY_H

#include "entities/packets/Packet.h"

#include "debug_root.h"

typedef enum {
    PACKET_TYPE_CREATE_ROUTING_TABLE,
    PACKET_TYPE_CREATE_ROUTING_PACKET,
    PACKET_TYPE_CREATE_CONTROL_PACKET,
    PACKET_TYPE_CREATE_EMPTY_CONTROL_PACKET,
    PACKET_TYPE_CREATE_DATA_PACKET,

} ePacketType_t;

class PacketFactory {
public:

    static void setMaxPacketSize(size_t setMaxPacketSize) {
        if (maxPacketSize == nullptr)
            maxPacketSize = new size_t(setMaxPacketSize);
        else
            *maxPacketSize = setMaxPacketSize;
    }

    static size_t getMaxPacketSize() {
        if (maxPacketSize == nullptr)
            return 0;
        return *maxPacketSize;
    }

    /**
     * @brief Create a T*
     *
     * @param payload Payload
     * @param payloadSize Length of the payload in bytes
     * @return T*
     */
    template<class T>
    static T* createPacket(uint8_t* payload, uint8_t payloadSize, uint8_t packet_type) {
        //Packet size = size of the header + size of the payload
        size_t packetSize = sizeof(T) + payloadSize;
        size_t maxPacketSize = PacketFactory::getMaxPacketSize();

        if (packetSize > maxPacketSize) {
            ESP_LOGW(LM_TAG, "Trying to create a packet greater than %d bytes", maxPacketSize);
            packetSize = maxPacketSize;
        }

        ESP_LOGV(LM_TAG, "Creating packet with %d bytes", packetSize);

        T* p = static_cast<T*>(pvPortMalloc(packetSize));

        if (!p) {
            switch (packet_type)
            {
                case PACKET_TYPE_CREATE_ROUTING_TABLE:
                    DebugHeapOnAllocationFail(ALLOCATION_CREATE_PACKET_ROUTING_TABLE, packetSize);
                    break;
                case PACKET_TYPE_CREATE_ROUTING_PACKET:
                    DebugHeapOnAllocationFail(ALLOCATION_CREATE_PACKET_ROUTING_PACKET, packetSize);
                    break;
                case PACKET_TYPE_CREATE_CONTROL_PACKET:
                    DebugHeapOnAllocationFail(ALLOCATION_CREATE_PACKET_CONTROL, packetSize);
                    break;
                case PACKET_TYPE_CREATE_EMPTY_CONTROL_PACKET:
                    DebugHeapOnAllocationFail(ALLOCATION_CREATE_PACKET_EMPTY_CONTROL, packetSize);
                    break;
                case PACKET_TYPE_CREATE_DATA_PACKET:
                    DebugHeapOnAllocationFail(ALLOCATION_CREATE_PACKET_DATA_PACKET, packetSize);
                    break;
                default:
                    DebugHeapOnAllocationFail(ALLOCATION_CREATE_PACKET_UNKNOWN_PACKET, packetSize);
                    break;
            }
            ESP_LOGE(LM_TAG, "Packet not allocated");
            return nullptr;
        }
        switch (packet_type)
        {
            case PACKET_TYPE_CREATE_ROUTING_TABLE:
                DebugHeapOnAllocation(ALLOCATION_CREATE_PACKET_ROUTING_TABLE, (void*)p, packetSize);
                break;
            case PACKET_TYPE_CREATE_ROUTING_PACKET:
                DebugHeapOnAllocation(ALLOCATION_CREATE_PACKET_ROUTING_PACKET, (void*)p, packetSize);
                break;
            case PACKET_TYPE_CREATE_CONTROL_PACKET:
                DebugHeapOnAllocation(ALLOCATION_CREATE_PACKET_CONTROL, (void*)p, packetSize);
                break;
            case PACKET_TYPE_CREATE_EMPTY_CONTROL_PACKET:
                DebugHeapOnAllocation(ALLOCATION_CREATE_PACKET_EMPTY_CONTROL, (void*)p, packetSize);
                break;
            case PACKET_TYPE_CREATE_DATA_PACKET:
                DebugHeapOnAllocation(ALLOCATION_CREATE_PACKET_DATA_PACKET, (void*)p, packetSize);
                break;
            default:
                DebugHeapOnAllocation(ALLOCATION_CREATE_PACKET_UNKNOWN_PACKET, (void*)p, packetSize);
                break;
        }
        


        if (payloadSize > 0) {
            //Copy the payload into the packet
            memcpy(reinterpret_cast<void*>((unsigned long) p + (sizeof(T))), payload, payloadSize);
        }

        ESP_LOGI(LM_TAG, "Packet created with %d bytes", packetSize);

        return p;
    };

private:
    static size_t* maxPacketSize;

};

#endif // _LORAMESHER_PACKET_FACTORY_H