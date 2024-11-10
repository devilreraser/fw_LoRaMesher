#ifndef DEBUG_HEAP
#define DEBUG_HEAP

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>


typedef enum {
    ALLOCATION_PRINTF_CIRCLE                = 0,
    ALLOCATION_PRINTF_QUEUE                 = 1,
    ALLOCATION_BIT_LIST                     = 2,
    ALLOCATION_APP_PACKET                   = 3,
    ALLOCATION_EMPTY_PACK                   = 4,
    ALLOCATION_APP_PACKET_CREATE_CONVERT    = 5,
    ALLOCATION_COPY_PACKET                  = 6,
    ALLOCATION_CREATE_PACKET_ROUTING_TABLE  = 7,
    ALLOCATION_CREATE_PACKET_ROUTING_PACKET = 8,
    PACKET_TYPE_CREATE_HELLO_PACKET         = 9,
    ALLOCATION_CREATE_PACKET_CONTROL        = 10,
    ALLOCATION_CREATE_PACKET_EMPTY_CONTROL  = 11,
    ALLOCATION_CREATE_PACKET_DATA_PACKET    = 12,
    ALLOCATION_CREATE_PACKET_UNKNOWN_PACKET = 13,
    ALLOCATION_COUNT

} e_AllocationName_t;



void DebugHeapInit(void);
void DebugHeapOnAllocation(e_AllocationName_t eName, void* pData, uint32_t nSize);
void DebugHeapOnFree(e_AllocationName_t eName, void* pData);
void DebugHeapOnFreeCheckAll(void* pData);
void DebugHeapPrint(bool skipUnused);
//void DebugHeapPrintPears(e_AllocationName_t eName);
void DebugHeapPrintPears(e_AllocationName_t eName, uint32_t maxDataLen);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_HEAP */