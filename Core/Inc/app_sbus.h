/* Platform-neutral SBUS pipe header (no HAL/RTOS types). */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t App_SbusPipe_Read(uint8_t* out, uint16_t max_len);
uint32_t App_SbusPipe_DroppedCount(void);
void App_SbusPipe_WaitRx(void);

// LiveWatch-friendly mirror of dropped counter (incremented in ISR push path).
extern volatile uint32_t g_app_sbus_pipe_dropped;

// LiveWatch-friendly RX counters (updated in USART3 RxEventCallback).
// - rx_events: how many times the UART IDLE/DMA event callback fired
// - rx_bytes:  how many bytes were pushed into the SBUS pipe total
extern volatile uint32_t g_app_sbus_rx_events;
extern volatile uint32_t g_app_sbus_rx_bytes;

#ifdef __cplusplus
}
#endif
