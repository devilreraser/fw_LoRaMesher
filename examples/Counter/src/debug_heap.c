/* Heap Leak Debugging Start */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#include "debug_heap.h"
#include "BuildOptions.h"

#define TAG "DBG_HEAP"

#define ALLOCATION_PAIR_MAX 32

typedef struct {
    uint32_t u32AllocatedBytesTotal;
    uint32_t u32FreedBytesTotal;
    uint32_t u32AllocatedBytesNow;
    uint32_t u32AllocatedTimesTotal;
    uint32_t u32FreedTimesTotal;
    uint32_t u32AllocatedTimesNow;
    uint32_t u32SingleAllocationBytesMax;
    uint32_t u32SingleFreeBytesMax;
    uint32_t u32SingleAllocationBytesMin;
    uint32_t u32SingleFreeBytesMin;
    
} s_AllocationData_t;

typedef struct {
    void* pData;
    uint32_t nSize;
} s_AllocationPair_t;

s_AllocationData_t asDebugHeapAllocation[ALLOCATION_COUNT] = {0};
s_AllocationPair_t asDebugHeapPair[ALLOCATION_COUNT][ALLOCATION_PAIR_MAX] = {0};
size_t nDebugHeapPairCount[ALLOCATION_COUNT] = {0};     /* should be same as u32AllocatedTimesNow */

void DebugHeapInit(void)
{
    for (int index = 0; index < ALLOCATION_COUNT; index++)
    {
        asDebugHeapAllocation->u32SingleAllocationBytesMax = 0;
        asDebugHeapAllocation->u32SingleFreeBytesMax = 0;
        asDebugHeapAllocation->u32SingleAllocationBytesMin = UINT32_MAX;
        asDebugHeapAllocation->u32SingleFreeBytesMin = UINT32_MAX;
        for (size_t j = 0; j < ALLOCATION_PAIR_MAX; j++) {
            asDebugHeapPair[index][j].nSize = 0;
            asDebugHeapPair[index][j].pData = NULL;
        }
        
    }
}

uint32_t findAllocationSize(e_AllocationName_t eName, void* pData) {
    if (pData == NULL) {return 0;}
    if (eName < ALLOCATION_COUNT) {
        for (size_t j = 0; j < ALLOCATION_PAIR_MAX; j++) {
            if (asDebugHeapPair[eName][j].pData == pData) {
                return asDebugHeapPair[eName][j].nSize;
            }
        }
    }
    // Address not found
    return 0;
}
void DebugHeapOnAllocation(e_AllocationName_t eName, void* pData, uint32_t nSize)
{
    if (pData == NULL || eName < ALLOCATION_COUNT)
    {
        asDebugHeapAllocation->u32AllocatedBytesTotal += nSize;
        asDebugHeapAllocation->u32AllocatedBytesNow += nSize;
        asDebugHeapAllocation->u32AllocatedTimesTotal++;
        asDebugHeapAllocation->u32AllocatedTimesNow++;
        if (asDebugHeapAllocation->u32SingleAllocationBytesMax < nSize)
        {
            asDebugHeapAllocation->u32SingleAllocationBytesMax = nSize;
        }
        if (asDebugHeapAllocation->u32SingleAllocationBytesMin > nSize)
        {
            asDebugHeapAllocation->u32SingleAllocationBytesMin = nSize;
        }

        // Get the current allocation count for this allocation name
        size_t* pairCount = &nDebugHeapPairCount[eName];
        // Add the allocation to the tracking array
        asDebugHeapPair[eName][*pairCount].pData = pData;
        asDebugHeapPair[eName][*pairCount].nSize = nSize;
        (*pairCount)++;  // Increment the allocation count

    }
    else {
        ESP_LOGV(TAG, "[ERROR] Invalid allocation input.");
        return;
    }
}

void DebugHeapOnFree(e_AllocationName_t eName, void* pData)
{
    if (pData == NULL || eName < ALLOCATION_COUNT)
    {
        // Find the allocation in the pair array
        uint32_t nBytes = 0;
        size_t* pairCount = &nDebugHeapPairCount[eName];
        for (size_t i = 0; i < *pairCount; i++) {
            if (asDebugHeapPair[eName][i].pData == pData) {
                nBytes = asDebugHeapPair[eName][i].nSize;

                // Shift the remaining entries in the pair array
                for (size_t j = i; j < *pairCount - 1; j++) {
                    asDebugHeapPair[eName][j] = asDebugHeapPair[eName][j + 1];
                }

                (*pairCount)--;  // Decrease the pair count

                break;
            }
        }

        asDebugHeapAllocation->u32FreedBytesTotal += nBytes;
        asDebugHeapAllocation->u32AllocatedBytesNow -= nBytes;
        asDebugHeapAllocation->u32FreedTimesTotal++;
        asDebugHeapAllocation->u32AllocatedTimesNow--;
        if (asDebugHeapAllocation->u32SingleFreeBytesMax < nBytes)
        {
            asDebugHeapAllocation->u32SingleFreeBytesMax = nBytes;
        }
        if (asDebugHeapAllocation->u32SingleFreeBytesMin > nBytes)
        {
            asDebugHeapAllocation->u32SingleFreeBytesMin = nBytes;
        }
    }
    else {
        ESP_LOGV(TAG, "[ERROR] Invalid free input.");
        return;
    }
}

//#define TAG "HeapDebug"

void DebugHeapPrint(void) {
    ESP_LOGE(TAG, "\n[Heap Debugging Info]\n");

    // Print compact header row
    ESP_LOGE(TAG, "-------------------------------------------------------------------------------------------------");
    ESP_LOGE(TAG, "| Type | TotAlloc | Freed  | CurrAlloc | MaxAlloc | MinAlloc | TotTimes | FreeTimes | CurrTimes |");
    ESP_LOGE(TAG, "-------------------------------------------------------------------------------------------------");

    for (int i = 0; i < ALLOCATION_COUNT; i++) {
        s_AllocationData_t* allocData = &asDebugHeapAllocation[i];
        
        ESP_LOGE(TAG, "| %-4d | %-8u | %-6u | %-9u | %-8u | %-8u | %-8u | %-9u | %-9u |",
                 i,
                 allocData->u32AllocatedBytesTotal,
                 allocData->u32FreedBytesTotal,
                 allocData->u32AllocatedBytesNow,
                 allocData->u32SingleAllocationBytesMax,
                 allocData->u32SingleAllocationBytesMin,
                 allocData->u32AllocatedTimesTotal,
                 allocData->u32FreedTimesTotal,
                 allocData->u32AllocatedTimesNow);
    }

    ESP_LOGE(TAG, "-------------------------------------------------------------------------------------------------");
    
    // Print active allocations for each allocation type
    #if 0
    ESP_LOGE(TAG, "\n[Active Allocations]\n");
    for (int i = 0; i < ALLOCATION_COUNT; i++) {
        ESP_LOGE(TAG, "Type %d Active Allocations:", i);
        for (size_t j = 0; j < nDebugHeapPairCount[i]; j++) {
            ESP_LOGE(TAG, "  %zu: Addr: %p, Size: %u bytes", j, asDebugHeapPair[i][j].pData, asDebugHeapPair[i][j].nSize);
        }
    }
    #endif
}

/* Heap Leak Debugging Final */

