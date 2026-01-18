/* Platform-neutral time port header (no HAL types). */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void App_Tim1_SchedulerStart(void);
uint64_t App_Tim1_LastTimestampUs(void);

#ifdef __cplusplus
}
#endif

