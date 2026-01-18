#include "app_spi.h"

#include "FreeRTOS.h"
#include "main.h"
#include "spi.h"
#include "task.h"

typedef struct
{
  TaskHandle_t waiting_task;
  volatile uint8_t in_progress;
  volatile uint8_t ok;
  uint8_t dev;
  App_OpCallback cb;
  void* cb_user;
} app_spi1_dma_ctx_t;

static app_spi1_dma_ctx_t g_spi1 = {0};

volatile uint32_t g_app_spi1_bmi088_txrx_ok = 0;
volatile uint32_t g_app_spi1_bmi088_txrx_fail = 0;

static void cs_low(uint8_t dev)
{
  if (dev == 0U)
    HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_RESET);
  else
    HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, GPIO_PIN_RESET);
}

static void cs_high(uint8_t dev)
{
  if (dev == 0U)
    HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, GPIO_PIN_SET);
}

static void short_delay(void)
{
  for (int i = 0; i < 300; ++i)
  {
    __NOP();
  }
}

static void spi1_on_complete_from_isr(uint8_t ok)
{
  if (!g_spi1.in_progress) return;

  cs_high(g_spi1.dev);
  g_spi1.ok = ok ? 1U : 0U;
  if (g_spi1.ok)
    g_app_spi1_bmi088_txrx_ok += 1U;
  else
    g_app_spi1_bmi088_txrx_fail += 1U;

  if (g_spi1.cb != NULL)
  {
    g_spi1.cb(g_spi1.ok, 1U, g_spi1.cb_user);
  }

  BaseType_t hpw = pdFALSE;
  if (g_spi1.waiting_task)
  {
    vTaskNotifyGiveFromISR(g_spi1.waiting_task, &hpw);
  }
  g_spi1.in_progress = 0U;
  portYIELD_FROM_ISR(hpw);
}

uint8_t App_Spi1_Bmi088_TxRxDmaOp(uint8_t dev, const uint8_t* tx, uint8_t* rx, uint16_t len,
                                  const App_Operation* op)
{
  if (tx == NULL || rx == NULL || len == 0U) return 0U;
  if (dev > 1U) return 0U;
  if (g_spi1.in_progress) return 0U;

  g_spi1.in_progress = 1U;
  g_spi1.ok = 0U;
  g_spi1.dev = dev;
  g_spi1.waiting_task = NULL;
  g_spi1.cb = NULL;
  g_spi1.cb_user = NULL;

  App_Operation local = App_OpNone();
  if (op != NULL) local = *op;
  if (local.mode == APP_OP_BLOCK)
  {
    g_spi1.waiting_task = xTaskGetCurrentTaskHandle();
  }
  else if (local.mode == APP_OP_CALLBACK)
  {
    g_spi1.cb = local.cb;
    g_spi1.cb_user = local.user;
  }

  cs_low(dev);
  short_delay();

  if (HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)tx, rx, len) != HAL_OK)
  {
    cs_high(dev);
    g_app_spi1_bmi088_txrx_fail += 1U;
    g_spi1.in_progress = 0U;
    return 0U;
  }

  if (local.mode == APP_OP_BLOCK)
  {
    const uint32_t timeout_ms = (local.timeout_ms == 0U) ? 20U : local.timeout_ms;
    const uint32_t taken = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms));
    if (taken == 0U || g_spi1.ok == 0U)
    {
      if (taken == 0U) g_app_spi1_bmi088_txrx_fail += 1U;
      cs_high(dev);
      (void)HAL_SPI_Abort_IT(&hspi1);
      g_spi1.in_progress = 0U;
      return 0U;
    }
    return 1U;
  }

  // APP_OP_NONE / APP_OP_CALLBACK: async fire-and-forget (completion happens via SPI callbacks).
  return 1U;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
  if (hspi->Instance == SPI1) spi1_on_complete_from_isr(1U);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
  if (hspi->Instance == SPI1) spi1_on_complete_from_isr(1U);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi)
{
  if (hspi->Instance == SPI1) spi1_on_complete_from_isr(0U);
}
