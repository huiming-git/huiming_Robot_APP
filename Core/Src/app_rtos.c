#include "app_rtos.h"

#include "FreeRTOS.h"
#include "task.h"

uint32_t App_Rtos_TickNow(void) { return (uint32_t)xTaskGetTickCount(); }

void App_Rtos_DelayMs(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

void App_Rtos_DelayUntilMs(uint32_t* last_wake_tick, uint32_t period_ms)
{
  if (last_wake_tick == NULL) return;
  TickType_t lw = (TickType_t)(*last_wake_tick);
  vTaskDelayUntil(&lw, pdMS_TO_TICKS(period_ms));
  *last_wake_tick = (uint32_t)lw;
}

