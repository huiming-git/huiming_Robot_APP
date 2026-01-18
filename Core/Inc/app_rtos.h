/* Platform-neutral RTOS port header (no FreeRTOS types). */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t App_Rtos_TickNow(void);
void App_Rtos_DelayMs(uint32_t ms);
void App_Rtos_DelayUntilMs(uint32_t* last_wake_tick, uint32_t period_ms);

#ifdef __cplusplus
}
#endif

