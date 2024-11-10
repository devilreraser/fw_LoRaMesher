#ifndef DEBUG_TASK
#define DEBUG_TASK

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>


void DebugTaskCounter(const char* name);
void DebugTaskPrintStats(void);
void DebugTaskResetStats(void);
#ifdef __cplusplus
}
#endif

#endif /* DEBUG_TASK */