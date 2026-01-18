#include "app_i2c.h"

#include "i2c.h"

volatile uint32_t g_app_i2c3_fail = 0;
volatile uint32_t g_app_i2c3_abort = 0;

typedef enum {
  APP_I2C3_STAGE_NONE = 0,
  APP_I2C3_STAGE_TX = 1,
  APP_I2C3_STAGE_RX = 2,
} app_i2c3_stage_t;

typedef struct {
  volatile uint8_t in_progress;
  volatile uint8_t ok;
  uint8_t addr_8bit;
  app_i2c3_stage_t stage;
  uint8_t* rx;
  uint16_t rx_len;
  App_OpCallback cb;
  void* cb_user;
  uint32_t start_ms;
  uint32_t timeout_ms;
} app_i2c3_ctx_t;

static app_i2c3_ctx_t g_i2c3 = {0};

static void i2c3_complete(uint8_t ok, uint8_t in_isr)
{
  if (!g_i2c3.in_progress) return;

  g_i2c3.ok = ok ? 1U : 0U;
  if (!g_i2c3.ok) g_app_i2c3_fail += 1U;

  if (g_i2c3.cb != NULL)
  {
    g_i2c3.cb(g_i2c3.ok, in_isr, g_i2c3.cb_user);
  }

  g_i2c3.in_progress = 0U;
  g_i2c3.stage = APP_I2C3_STAGE_NONE;
  g_i2c3.rx = NULL;
  g_i2c3.rx_len = 0U;
  g_i2c3.cb = NULL;
  g_i2c3.cb_user = NULL;
  g_i2c3.start_ms = 0U;
  g_i2c3.timeout_ms = 0U;
}

uint8_t App_I2c3_Write(uint8_t addr_8bit, const uint8_t* data, uint16_t len, uint32_t timeout_ms)
{
  if (data == NULL || len == 0U) return 0U;
  const uint8_t ok =
      (HAL_I2C_Master_Transmit(&hi2c3, addr_8bit, (uint8_t*)data, len, timeout_ms) == HAL_OK) ? 1U
                                                                                               : 0U;
  if (!ok) g_app_i2c3_fail += 1U;
  return ok;
}

uint8_t App_I2c3_Read(uint8_t addr_8bit, uint8_t* data, uint16_t len, uint32_t timeout_ms)
{
  if (data == NULL || len == 0U) return 0U;
  const uint8_t ok =
      (HAL_I2C_Master_Receive(&hi2c3, addr_8bit, data, len, timeout_ms) == HAL_OK) ? 1U : 0U;
  if (!ok) g_app_i2c3_fail += 1U;
  return ok;
}

uint8_t App_I2c3_WriteRead(uint8_t addr_8bit, const uint8_t* tx, uint16_t tx_len, uint8_t* rx,
                           uint16_t rx_len, uint32_t timeout_ms)
{
  if (tx == NULL || tx_len == 0U) return 0U;
  if (HAL_I2C_Master_Transmit(&hi2c3, addr_8bit, (uint8_t*)tx, tx_len, timeout_ms) != HAL_OK)
  {
    g_app_i2c3_fail += 1U;
    return 0U;
  }
  if (rx == NULL || rx_len == 0U) return 1U;
  const uint8_t ok =
      (HAL_I2C_Master_Receive(&hi2c3, addr_8bit, rx, rx_len, timeout_ms) == HAL_OK) ? 1U : 0U;
  if (!ok) g_app_i2c3_fail += 1U;
  return ok;
}

static uint8_t i2c3_start_async(uint8_t addr_8bit, const uint8_t* tx, uint16_t tx_len, uint8_t* rx,
                                uint16_t rx_len, const App_Operation* op)
{
  if (g_i2c3.in_progress) return 0U;

  App_Operation local = App_OpNone();
  if (op != NULL) local = *op;
  if (local.mode != APP_OP_NONE && local.mode != APP_OP_CALLBACK) return 0U;

  g_i2c3.in_progress = 1U;
  g_i2c3.ok = 0U;
  g_i2c3.addr_8bit = addr_8bit;
  g_i2c3.stage = APP_I2C3_STAGE_NONE;
  g_i2c3.rx = rx;
  g_i2c3.rx_len = rx_len;
  g_i2c3.cb = (local.mode == APP_OP_CALLBACK) ? local.cb : NULL;
  g_i2c3.cb_user = (local.mode == APP_OP_CALLBACK) ? local.user : NULL;
  g_i2c3.start_ms = HAL_GetTick();
  g_i2c3.timeout_ms = (local.timeout_ms == 0U) ? 20U : local.timeout_ms;

  if (tx != NULL && tx_len != 0U)
  {
    g_i2c3.stage = APP_I2C3_STAGE_TX;
    if (HAL_I2C_Master_Transmit_DMA(&hi2c3, addr_8bit, (uint8_t*)tx, tx_len) != HAL_OK)
    {
      g_app_i2c3_fail += 1U;
      g_i2c3.in_progress = 0U;
      g_i2c3.stage = APP_I2C3_STAGE_NONE;
      g_i2c3.rx = NULL;
      g_i2c3.rx_len = 0U;
      g_i2c3.cb = NULL;
      g_i2c3.cb_user = NULL;
      return 0U;
    }
    return 1U;
  }

  // Read-only.
  g_i2c3.stage = APP_I2C3_STAGE_RX;
  if (HAL_I2C_Master_Receive_DMA(&hi2c3, addr_8bit, rx, rx_len) != HAL_OK)
  {
    g_app_i2c3_fail += 1U;
    g_i2c3.in_progress = 0U;
    g_i2c3.stage = APP_I2C3_STAGE_NONE;
    g_i2c3.rx = NULL;
    g_i2c3.rx_len = 0U;
    g_i2c3.cb = NULL;
    g_i2c3.cb_user = NULL;
    return 0U;
  }
  return 1U;
}

uint8_t App_I2c3_WriteOp(uint8_t addr_8bit, const uint8_t* data, uint16_t len,
                         const App_Operation* op)
{
  if (data == NULL || len == 0U) return 0U;

  App_Operation local = App_OpNone();
  if (op != NULL) local = *op;

  if (local.mode == APP_OP_BLOCK)
  {
    if (g_i2c3.in_progress) return 0U;
    g_i2c3.in_progress = 1U;
    const uint32_t to = (local.timeout_ms == 0U) ? 20U : local.timeout_ms;
    const uint8_t ok = App_I2c3_Write(addr_8bit, data, len, to);
    g_i2c3.in_progress = 0U;
    return ok;
  }

  return i2c3_start_async(addr_8bit, data, len, NULL, 0U, &local);
}

uint8_t App_I2c3_ReadOp(uint8_t addr_8bit, uint8_t* data, uint16_t len, const App_Operation* op)
{
  if (data == NULL || len == 0U) return 0U;

  App_Operation local = App_OpNone();
  if (op != NULL) local = *op;

  if (local.mode == APP_OP_BLOCK)
  {
    if (g_i2c3.in_progress) return 0U;
    g_i2c3.in_progress = 1U;
    const uint32_t to = (local.timeout_ms == 0U) ? 20U : local.timeout_ms;
    const uint8_t ok = App_I2c3_Read(addr_8bit, data, len, to);
    g_i2c3.in_progress = 0U;
    return ok;
  }

  return i2c3_start_async(addr_8bit, NULL, 0U, data, len, &local);
}

uint8_t App_I2c3_WriteReadOp(uint8_t addr_8bit, const uint8_t* tx, uint16_t tx_len, uint8_t* rx,
                             uint16_t rx_len, const App_Operation* op)
{
  if (tx == NULL || tx_len == 0U) return 0U;

  App_Operation local = App_OpNone();
  if (op != NULL) local = *op;

  if (local.mode == APP_OP_BLOCK)
  {
    if (g_i2c3.in_progress) return 0U;
    g_i2c3.in_progress = 1U;
    const uint32_t to = (local.timeout_ms == 0U) ? 20U : local.timeout_ms;
    const uint8_t ok = App_I2c3_WriteRead(addr_8bit, tx, tx_len, rx, rx_len, to);
    g_i2c3.in_progress = 0U;
    return ok;
  }

  if (rx == NULL || rx_len == 0U)
  {
    return i2c3_start_async(addr_8bit, tx, tx_len, NULL, 0U, &local);
  }

  return i2c3_start_async(addr_8bit, tx, tx_len, rx, rx_len, &local);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
  if (hi2c != &hi2c3) return;
  if (!g_i2c3.in_progress) return;
  if (g_i2c3.stage != APP_I2C3_STAGE_TX) return;

  if (g_i2c3.rx != NULL && g_i2c3.rx_len != 0U)
  {
    g_i2c3.stage = APP_I2C3_STAGE_RX;
    if (HAL_I2C_Master_Receive_DMA(&hi2c3, g_i2c3.addr_8bit, g_i2c3.rx, g_i2c3.rx_len) != HAL_OK)
    {
      i2c3_complete(0U, 1U);
    }
    return;
  }

  i2c3_complete(1U, 1U);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
  if (hi2c != &hi2c3) return;
  i2c3_complete(1U, 1U);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c)
{
  if (hi2c != &hi2c3) return;
  i2c3_complete(0U, 1U);
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef* hi2c)
{
  if (hi2c != &hi2c3) return;
  i2c3_complete(0U, 1U);
}

void App_I2c3_Service(void)
{
  if (!g_i2c3.in_progress) return;
  if (g_i2c3.timeout_ms == 0U) return;

  const uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - g_i2c3.start_ms) < g_i2c3.timeout_ms) return;

  // Timeout: request abort. Completion callback will clear state.
  g_app_i2c3_abort += 1U;
  if (HAL_I2C_Master_Abort_IT(&hi2c3, (uint16_t)g_i2c3.addr_8bit) != HAL_OK)
  {
    // If abort request fails, force-clear to avoid permanent lockup.
    i2c3_complete(0U, 0U);
  }
}

void App_DelayMs(uint32_t ms) { HAL_Delay(ms); }
