#ifndef DEBUG_HEAP
#define DEBUG_HEAP

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define USE_ALLOCATION_PRINTF_CIRCLE    0
#define USE_ALLOCATION_PRINTF_QUEUE     0
#define USE_ALLOCATION_APP_PACKET       0
#define USE_ALLOCATION_COPY_PACKET      0
#define USE_ALLOCATION_ROUTING_TABLE    1
#define USE_ALLOCATION_ROUTING_PACKET   0
#define USE_ALLOCATION_CONTROL_PACKET   0
#define USE_ALLOCATION_CONTROL_EMPTY    0
#define USE_ALLOCATION_UNKNOWN_PACKET   0


typedef enum {
    #if USE_ALLOCATION_PRINTF_CIRCLE
    ALLOCATION_PRINTF_CIRCLE               ,
    #endif
    #if USE_ALLOCATION_PRINTF_QUEUE
    ALLOCATION_PRINTF_QUEUE                ,
    #endif
    ALLOCATION_BIT_LIST                    ,
    #if USE_ALLOCATION_APP_PACKET
    ALLOCATION_APP_PACKET                  ,
    #endif
    ALLOCATION_EMPTY_PACK                  ,
    ALLOCATION_APP_PACKET_CREATE_CONVERT   ,
    #if USE_ALLOCATION_COPY_PACKET
    ALLOCATION_COPY_PACKET                 ,
    #endif
    #if USE_ALLOCATION_ROUTING_TABLE
    ALLOCATION_CREATE_PACKET_ROUTING_TABLE ,
    #endif
    #if USE_ALLOCATION_ROUTING_PACKET
    ALLOCATION_CREATE_PACKET_ROUTING_PACKET,
    #endif
    ALLOCATION_CREATE_PACKET_HELLO         ,
    #if USE_ALLOCATION_CONTROL_PACKET
    ALLOCATION_CREATE_PACKET_CONTROL       ,
    #endif
    #if USE_ALLOCATION_CONTROL_EMPTY
    ALLOCATION_CREATE_PACKET_EMPTY_CONTROL ,
    #endif
    ALLOCATION_CREATE_PACKET_DATA_PACKET   ,
    #if USE_ALLOCATION_UNKNOWN_PACKET
    ALLOCATION_CREATE_PACKET_UNKNOWN_PACKET,
    #endif
    ALLOCATION_CREATE_ROUTE_NODE,
    ALLOCATION_COUNT

} e_AllocationName_t;



void DebugHeapInit(void);
void DebugHeapOnAllocationFail(e_AllocationName_t eName, uint32_t nSize);
void DebugHeapOnAllocation(e_AllocationName_t eName, void* pData, uint32_t nSize);
void DebugHeapOnFree(e_AllocationName_t eName, void* pData);
void DebugHeapOnFreeCheckAll(void* pData);
void DebugHeapPrint(bool skipUnused);
void DebugHeapPrintPears(e_AllocationName_t eName, uint32_t maxDataLen);
void DebugHeapOnAllocationSkipPrintfCircle(void);
void DebugHeapOnAllocationSkipPrintfQueue(void);
void DebugHeapOnAllocationSkipAppPacket(void);
void DebugHeapOnAllocationSkipCopyPacket(void);
void DebugHeapOnAllocationSkipRoutingTable(void);
void DebugHeapOnAllocationSkipRoutingPacket(void);
void DebugHeapOnAllocationSkipControlPacket(void);
void DebugHeapOnAllocationSkipControlEmpty(void);
void DebugHeapOnAllocationSkipUnknownPacket(void);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_HEAP */