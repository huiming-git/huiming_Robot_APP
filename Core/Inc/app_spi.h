/* Platform-neutral SPI port header (no HAL types). */
#pragma once

#include <stdint.h>

#include "app_op.h"

#ifdef __cplusplus
extern "C" {
#endif

// BMI088 is wired on SPI1 with two chip selects. dev: 0=accel, 1=gyro.
uint8_t App_Spi1_Bmi088_TxRxDmaOp(uint8_t dev, const uint8_t* tx, uint8_t* rx, uint16_t len,
                                  const App_Operation* op);

// LiveWatch-friendly diagnostics.
extern volatile uint32_t g_app_spi1_bmi088_txrx_ok;
extern volatile uint32_t g_app_spi1_bmi088_txrx_fail;

static inline uint8_t App_Spi1_Bmi088_TxRxDma(uint8_t dev, const uint8_t* tx, uint8_t* rx,
                                              uint16_t len, uint32_t timeout_ms)
{
  App_Operation op = App_OpBlock(timeout_ms);
  return App_Spi1_Bmi088_TxRxDmaOp(dev, tx, rx, len, &op);
}

#ifdef __cplusplus
}
#endif
