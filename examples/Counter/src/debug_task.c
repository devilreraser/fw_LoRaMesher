#include "debug_task.h"
#include "BuildOptions.h"

#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define MAX_TASKS 16
#define TASK_NAME_LEN 16
#define TAG "DBG_TASK"

typedef struct {
    char taskName[TASK_NAME_LEN];
    uint32_t loopCount;
    uint32_t prevLoopCount;  // For calculating difference
} TaskLoopCounter;

static TaskLoopCounter taskLoopCounters[MAX_TASKS] = {0};
static SemaphoreHandle_t counterMutex = NULL;
static bool initFailed = false; // Track initialization failure to prevent repeated logs.

/**
 * @brief Initializes the task loop counter module.
 */
void DebugTaskInit() {
    if (counterMutex == NULL && !initFailed) {
        counterMutex = xSemaphoreCreateMutex();
        if (counterMutex == NULL) {
            ESP_LOGE(TAG, "[ERROR] Failed to create counter mutex!");
            initFailed = true;
        }
    }
}

/**
 * @brief Tracks the number of iterations for a specific task.
 *
 * @param name Name of the task.
 */
void DebugTaskCounter(const char* name) {
    if (counterMutex == NULL) {
        DebugTaskInit();
    }

    if (counterMutex != NULL) {
        xSemaphoreTake(counterMutex, portMAX_DELAY);

        for (int i = 0; i < MAX_TASKS; i++) {
            if (strncmp(taskLoopCounters[i].taskName, name, TASK_NAME_LEN) == 0) {
                taskLoopCounters[i].loopCount++;
                xSemaphoreGive(counterMutex);
                return;
            }
        }

        for (int i = 0; i < MAX_TASKS; i++) {
            if (taskLoopCounters[i].taskName[0] == '\0') {
                strncpy(taskLoopCounters[i].taskName, name, TASK_NAME_LEN - 1);
                taskLoopCounters[i].taskName[TASK_NAME_LEN - 1] = '\0';
                taskLoopCounters[i].loopCount = 1;
                taskLoopCounters[i].prevLoopCount = 0;
                xSemaphoreGive(counterMutex);
                return;
            }
        }

        ESP_LOGE(TAG, "[ERROR] TaskLoopCounters array is full!");
        xSemaphoreGive(counterMutex);
    }
}

/**
 * @brief Prints task loop statistics.
 */
void DebugTaskPrintStats() {
    if (counterMutex == NULL) {
        DebugTaskInit();
    }

    if (counterMutex != NULL) {
        xSemaphoreTake(counterMutex, portMAX_DELAY);

        ESP_LOGE(TAG, "\n[Task Loop Stats]");
        ESP_LOGE(TAG, "-----------------------------------------");
        ESP_LOGE(TAG, "| Task Name       | Loop Count | Diff   |");
        ESP_LOGE(TAG, "-----------------------------------------");

        for (int i = 0; i < MAX_TASKS; i++) {
            if (taskLoopCounters[i].taskName[0] != '\0') {
                uint32_t diff = taskLoopCounters[i].loopCount - taskLoopCounters[i].prevLoopCount;
                ESP_LOGE(TAG, "| %-15s | %-10u | %-6u |", 
                    taskLoopCounters[i].taskName, 
                    taskLoopCounters[i].loopCount, 
                    diff);
                taskLoopCounters[i].prevLoopCount = taskLoopCounters[i].loopCount;  // Update for next diff
            }
        }

        ESP_LOGE(TAG, "-----------------------------------------");

        xSemaphoreGive(counterMutex);
    }
}

/**
 * @brief Resets all task loop counters.
 */
void DebugTaskResetStats() {
    if (counterMutex == NULL) {
        DebugTaskInit();
    }

    if (counterMutex != NULL) {
        xSemaphoreTake(counterMutex, portMAX_DELAY);
        memset(taskLoopCounters, 0, sizeof(taskLoopCounters));
        ESP_LOGE(TAG, "[INFO] Task counters reset.");
        xSemaphoreGive(counterMutex);
    }
}
