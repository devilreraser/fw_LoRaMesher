#ifndef DEBUG_HEAP
#define DEBUG_HEAP

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


typedef enum {
    ALLOCATION_PRINTF_CIRCLE,
    ALLOCATION_PRINTF_QUEUE,
    ALLOCATION_BIT_LIST,
    ALLOCATION_COUNT

} e_AllocationName_t;


void DebugHeapInit(void);
void DebugHeapOnAllocation(e_AllocationName_t eName, void* pData, uint32_t nSize);
void DebugHeapOnFree(e_AllocationName_t eName, void* pData);
void DebugHeapPrint(void);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_HEAP */