/* Heap Leak Debugging Start */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#include "debug_heap.h"
#include "BuildOptions.h"

#define TAG "DBG_HEAP"

#define ALLOCATION_PAIR_MAX 32

//#define SEMAPHORE_WAIT_TICKS    pdMS_TO_TICKS(10)
#define SEMAPHORE_WAIT_TICKS    portMAX_DELAY
//#define SEMAPHORE_WAIT_TICKS    0

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
    size_t nPosAllocate;
} s_AllocationPair_t;

s_AllocationData_t asDebugHeapAllocation[ALLOCATION_COUNT] = {0};
s_AllocationPair_t asDebugHeapPair[ALLOCATION_COUNT][ALLOCATION_PAIR_MAX] = {0};
size_t nDebugHeapPairCount[ALLOCATION_COUNT] = {0};     /* should be same as u32AllocatedTimesNow */
size_t nNotMonitoredMemoryFreeTimes = 0;

uint32_t u32DebugHeapOnAllocationSkipped[ALLOCATION_COUNT] = {0};
uint32_t u32DebugHeapOnAllocationFail[ALLOCATION_COUNT] = {0};
uint32_t u32DebugHeapOnFreeSkipped[ALLOCATION_COUNT] = {0};
uint32_t u32DebugHeapOnFreeSkippedCheckAll = 0;

uint32_t u32OnAllocationSkipPrintfCircle = 0;
uint32_t u32OnAllocationSkipPrintfQueue = 0;




SemaphoreHandle_t semphrDebugHeapBusy = NULL;

void DebugHeapInit(void)
{
    semphrDebugHeapBusy = xSemaphoreCreateBinary();
    xSemaphoreGive(semphrDebugHeapBusy);
    for (int index = 0; index < ALLOCATION_COUNT; index++)
    {
        asDebugHeapAllocation[index].u32SingleAllocationBytesMax = 0;
        asDebugHeapAllocation[index].u32SingleFreeBytesMax = 0;
        asDebugHeapAllocation[index].u32SingleAllocationBytesMin = UINT32_MAX;
        asDebugHeapAllocation[index].u32SingleFreeBytesMin = UINT32_MAX;
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

void DebugHeapOnAllocationFail(e_AllocationName_t eName, uint32_t nSize)
{
    if (eName < ALLOCATION_COUNT)
    {
        u32DebugHeapOnAllocationFail[eName]++;
    }
}

void DebugHeapOnAllocation(e_AllocationName_t eName, void* pData, uint32_t nSize)
{
    if (pData == NULL || eName < ALLOCATION_COUNT)
    {
        if (xSemaphoreTake(semphrDebugHeapBusy, SEMAPHORE_WAIT_TICKS) == pdFALSE)
        {
            u32DebugHeapOnAllocationSkipped[eName]++;
        }
        else
        {
            // Get the current allocation count for this allocation name
            size_t* pairCount = &nDebugHeapPairCount[eName];
            // Add the allocation to the tracking array
            asDebugHeapPair[eName][*pairCount].pData = pData;
            asDebugHeapPair[eName][*pairCount].nSize = nSize;
            asDebugHeapPair[eName][*pairCount].nPosAllocate = asDebugHeapAllocation[eName].u32AllocatedTimesTotal;
            (*pairCount)++;  // Increment the allocation count

            asDebugHeapAllocation[eName].u32AllocatedBytesTotal += nSize;
            asDebugHeapAllocation[eName].u32AllocatedBytesNow += nSize;
            asDebugHeapAllocation[eName].u32AllocatedTimesTotal++;
            asDebugHeapAllocation[eName].u32AllocatedTimesNow++;
            if (asDebugHeapAllocation[eName].u32SingleAllocationBytesMax < nSize)
            {
                asDebugHeapAllocation[eName].u32SingleAllocationBytesMax = nSize;
            }
            if (asDebugHeapAllocation[eName].u32SingleAllocationBytesMin > nSize)
            {
                asDebugHeapAllocation[eName].u32SingleAllocationBytesMin = nSize;
            }
            
            xSemaphoreGive(semphrDebugHeapBusy);
        }

    }
    else {
        // Free the allocation if found
        //ESP_LOGV(TAG, "[ERROR] Invalid allocation input.");
        return;
    }
}

void DebugHeapOnFree(e_AllocationName_t eName, void* pData)
{
    if (pData == NULL || eName < ALLOCATION_COUNT)
    {
        if (xSemaphoreTake(semphrDebugHeapBusy, SEMAPHORE_WAIT_TICKS) == pdFALSE)
        {
            u32DebugHeapOnFreeSkipped[eName]++;
        }
        else
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

            asDebugHeapAllocation[eName].u32FreedBytesTotal += nBytes;
            asDebugHeapAllocation[eName].u32AllocatedBytesNow -= nBytes;
            asDebugHeapAllocation[eName].u32FreedTimesTotal++;
            asDebugHeapAllocation[eName].u32AllocatedTimesNow--;
            if (asDebugHeapAllocation[eName].u32SingleFreeBytesMax < nBytes)
            {
                asDebugHeapAllocation[eName].u32SingleFreeBytesMax = nBytes;
            }
            if (asDebugHeapAllocation[eName].u32SingleFreeBytesMin > nBytes)
            {
                asDebugHeapAllocation[eName].u32SingleFreeBytesMin = nBytes;
            }
            xSemaphoreGive(semphrDebugHeapBusy);
        }
    }
    else {
        // Free the allocation if found
        //ESP_LOGV(TAG, "[ERROR] Invalid free input.");
        return;
    }
}


void DebugHeapOnFreeCheckAll(void* pData) {
    if (pData == NULL) {
        // Free the allocation if found
        ESP_LOGE(TAG, "[ERROR] NULL pointer passed to DebugHeapOnFreeCheckAll.");
        return;
    }


    for (int i = 0; i < ALLOCATION_COUNT; i++) {
        size_t* pairCount = &nDebugHeapPairCount[i];
        for (size_t j = 0; j < *pairCount; j++) {
            if (asDebugHeapPair[i][j].pData == pData) {
                // Free the allocation if found
                DebugHeapOnFree((e_AllocationName_t)i, pData);

                //do not print here - could block forever on debug heap
                ESP_LOGV(TAG, "Freed allocation: Type %d, Addr: %p, Size: %u bytes", i, pData, asDebugHeapPair[i][j].nSize);

                return; // Exit once the allocation is found and freed
            }
        }
    }

    // Free the allocation if found
    ESP_LOGE(TAG, "Pointer %p not found in any allocation type.", pData);
}



//#define TAG "HeapDebug"

void DebugHeapPrint(bool skipUnused) {
    ESP_LOGE(TAG, "\n[Heap Debugging Info]\n");

    // Print compact header row
    ESP_LOGE(TAG, "------------------------------------------------------------------------------");
    ESP_LOGE(TAG, "| Type | TotAlloc | Freed  | Curr | Max | Min | Times | Free | Now | A/F Fail|");
    ESP_LOGE(TAG, "------------------------------------------------------------------------------");

    for (int i = 0; i < ALLOCATION_COUNT; i++) {
        s_AllocationData_t* allocData = &asDebugHeapAllocation[i];
        
        if (skipUnused == false || (int)allocData->u32SingleAllocationBytesMin >= 0)
        ESP_LOGE(TAG, "| %-4d | %-8u | %-6u | %-4u | %-3u | %-3d | %-5u | %-4u | %-3u | %-2u/%-2u %-2u|",
                 i,
                 allocData->u32AllocatedBytesTotal,
                 allocData->u32FreedBytesTotal,
                 allocData->u32AllocatedBytesNow,
                 allocData->u32SingleAllocationBytesMax,
                 (int)allocData->u32SingleAllocationBytesMin,
                 allocData->u32AllocatedTimesTotal,
                 allocData->u32FreedTimesTotal,
                 allocData->u32AllocatedTimesNow,
                 u32DebugHeapOnAllocationSkipped[i],
                 u32DebugHeapOnFreeSkipped[i],
                 u32DebugHeapOnAllocationFail[i]
                 );
    }
    ESP_LOGE(TAG, "------------------------------------------------------------------------------");
    ESP_LOGE(TAG, "nNotMonitoredMemoryFreeTimes:        %d", nNotMonitoredMemoryFreeTimes);
    ESP_LOGE(TAG, "u32DebugHeapOnFreeSkippedCheckAll:   %d", u32DebugHeapOnFreeSkippedCheckAll);
    ESP_LOGE(TAG, "u32OnAllocationSkipPrintfCircle:     %d", u32OnAllocationSkipPrintfCircle);
    ESP_LOGE(TAG, "u32OnAllocationSkipPrintfQueue:      %d", u32OnAllocationSkipPrintfQueue);

}

void DebugHeapPrintPears(e_AllocationName_t eName, uint32_t maxDataLen) {
    if (eName >= ALLOCATION_COUNT) {
        ESP_LOGE(TAG, "[ERROR] Invalid allocation type: %d", (int)eName);
        return;
    }

    // Print a compact header
    ESP_LOGE(TAG, "\n[Active Allocations for Type %d]\n", (int)eName);
    ESP_LOGE(TAG, "------------------------------------------------------------------------------------------------------------------------------");
    ESP_LOGE(TAG, "| Address  | Len | Pos | 00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 ... |");
    ESP_LOGE(TAG, "------------------------------------------------------------------------------------------------------------------------------");

    // Loop through active allocation pairs for the given type
    for (size_t j = 0; j < nDebugHeapPairCount[eName]; j++) {
        void* pData = asDebugHeapPair[eName][j].pData;
        uint8_t* byteData = (uint8_t*)(pData);
        uint32_t nSize = asDebugHeapPair[eName][j].nSize;
        size_t printLen = (nSize < maxDataLen) ? nSize : maxDataLen;

        // Prepare the data string
        char dataBuffer[128] = {0};  // Buffer for 32 bytes (formatted as "XX ") plus null terminator
        size_t pos = 0;
        for (size_t k = 0; k < printLen && pos < sizeof(dataBuffer) - 3; k++) {
            pos += snprintf(&dataBuffer[pos], sizeof(dataBuffer) - pos, "%02X ", byteData[k]);
        }

        if (printLen < nSize) {
            snprintf(&dataBuffer[pos], sizeof(dataBuffer) - pos, "...");
        }

        // Print the formatted line for the allocation
        ESP_LOGE(TAG, "| %08X | %-3u | %-3u | %-99s |", (uintptr_t)pData, nSize, asDebugHeapPair[eName][j].nPosAllocate, dataBuffer);
    }

    ESP_LOGE(TAG, "------------------------------------------------------------------------------------------------------------------------------");
}

void DebugHeapOnAllocationSkipPrintfCircle(void)
{
    u32OnAllocationSkipPrintfCircle++;
}
void DebugHeapOnAllocationSkipPrintfQueue(void)
{
    u32OnAllocationSkipPrintfQueue++;
}

/* Heap Leak Debugging Final */

