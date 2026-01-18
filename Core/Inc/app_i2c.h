/* Platform-neutral I2C port header (no HAL types). */
#pragma once

#include <stdint.h>

#include "app_op.h"

#ifdef __cplusplus
extern "C" {
#endif

uint8_t App_I2c3_Write(uint8_t addr_8bit, const uint8_t* data, uint16_t len, uint32_t timeout_ms);
uint8_t App_I2c3_Read(uint8_t addr_8bit, uint8_t* data, uint16_t len, uint32_t timeout_ms);
uint8_t App_I2c3_WriteRead(uint8_t addr_8bit, const uint8_t* tx, uint16_t tx_len, uint8_t* rx,
                           uint16_t rx_len, uint32_t timeout_ms);

// Operation-aware variants:
// - BLOCK: completes before returning (uses timeout_ms in op, defaults applied by implementation)
// - CALLBACK: invokes op->cb(ok, in_isr, user) on completion, returns once started
// - NONE: fire-and-forget, returns once started
uint8_t App_I2c3_WriteOp(uint8_t addr_8bit, const uint8_t* data, uint16_t len,
                         const App_Operation* op);
uint8_t App_I2c3_ReadOp(uint8_t addr_8bit, uint8_t* data, uint16_t len, const App_Operation* op);
uint8_t App_I2c3_WriteReadOp(uint8_t addr_8bit, const uint8_t* tx, uint16_t tx_len, uint8_t* rx,
                             uint16_t rx_len, const App_Operation* op);
void App_DelayMs(uint32_t ms);

// Service function for async operations (CALLBACK/NONE): enforces timeouts and requests abort.
// Call periodically from a task context (e.g. sensor task at 200Hz).
void App_I2c3_Service(void);

// LiveWatch-friendly diagnostics.
extern volatile uint32_t g_app_i2c3_fail;
extern volatile uint32_t g_app_i2c3_abort;

#ifdef __cplusplus
}
#endif
