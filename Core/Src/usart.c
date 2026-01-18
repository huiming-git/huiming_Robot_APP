/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "cmsis_os.h"
#include "app_bridge.h"
#include "main.h"
#include "app_sbus.h"

#define SBUS_RX_BUF_LEN 64
static uint8_t sbus_rx_buf[SBUS_RX_BUF_LEN];
static volatile uint16_t sbus_rx_last_idx = 0;

// Lock-free SBUS byte pipe: ISR pushes, sbusTask pops.
#define SBUS_PIPE_CAPACITY 512
static uint8_t sbus_pipe_buf[SBUS_PIPE_CAPACITY];

static volatile uint16_t sbus_pipe_w = 0;
static volatile uint16_t sbus_pipe_r = 0;
volatile uint32_t g_app_sbus_pipe_dropped = 0;
volatile uint32_t g_app_sbus_rx_events = 0;
volatile uint32_t g_app_sbus_rx_bytes = 0;

extern osThreadId_t sbusTaskHandle;

static inline uint16_t sbus_pipe_next(uint16_t idx)
{
  return (uint16_t)((idx + 1U) % SBUS_PIPE_CAPACITY);
}

static void sbus_pipe_push_from_isr(const uint8_t* data, uint16_t len)
{
  if (data == NULL || len == 0U) return;

  uint16_t w = sbus_pipe_w;
  const uint16_t r = sbus_pipe_r;
  for (uint16_t i = 0; i < len; ++i)
  {
    const uint16_t next = sbus_pipe_next(w);
    if (next == r)
    {
      g_app_sbus_pipe_dropped += 1U;
      continue;
    }
    sbus_pipe_buf[w] = data[i];
    w = next;
  }
  __DMB();
  sbus_pipe_w = w;
}

/* USER CODE END 0 */

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 100000;
  huart3.Init.WordLength = UART_WORDLENGTH_9B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_EVEN;
  huart3.Init.Mode = UART_MODE_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  // 启动 SBUS RX DMA + IDLE，关闭半传输中断
  sbus_rx_last_idx = 0;
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, sbus_rx_buf, SBUS_RX_BUF_LEN);
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PC11     ------> USART3_RX
    PC10     ------> USART3_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_RX Init */
    hdma_usart3_rx.Instance = DMA1_Stream1;
    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PC11     ------> USART3_RX
    PC10     ------> USART3_TX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_11|GPIO_PIN_10);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART3)
  {
    // NOTE:
    // In DMA circular mode, some HAL variants may report `Size` as a running byte count rather than
    // a stable index within the DMA buffer. If `Size` exceeds SBUS_RX_BUF_LEN, the old logic
    // `while (idx != Size)` can become an infinite loop inside ISR and freeze the whole system.
    // To be robust, derive the current write index directly from the DMA NDTR counter.
    (void)Size;
    uint16_t current = (uint16_t)(SBUS_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx));
    current = (uint16_t)(current % SBUS_RX_BUF_LEN);
    uint16_t idx = sbus_rx_last_idx;
    uint16_t pushed = 0;
    while (idx != current)
    {
      sbus_pipe_push_from_isr(&sbus_rx_buf[idx], 1);
      pushed += 1U;
      idx = (uint16_t)((idx + 1U) % SBUS_RX_BUF_LEN);
    }
    sbus_rx_last_idx = current;

    g_app_sbus_rx_events += 1U;
    g_app_sbus_rx_bytes += (uint32_t)pushed;

    if (sbusTaskHandle != NULL)
    {
      (void)osThreadFlagsSet(sbusTaskHandle, APP_SBUS_RX_FLAG);
    }
  }
}

uint16_t App_SbusPipe_Read(uint8_t* out, uint16_t max_len)
{
  if (out == NULL || max_len == 0U) return 0U;

  uint16_t r = sbus_pipe_r;
  const uint16_t w = sbus_pipe_w;
  if (r == w) return 0U;

  uint16_t n = 0;
  while (n < max_len && r != w)
  {
    out[n++] = sbus_pipe_buf[r];
    r = sbus_pipe_next(r);
  }
  __DMB();
  sbus_pipe_r = r;
  return n;
}

uint32_t App_SbusPipe_DroppedCount(void) { return g_app_sbus_pipe_dropped; }

void App_SbusPipe_WaitRx(void)
{
  (void)osThreadFlagsWait(APP_SBUS_RX_FLAG, osFlagsWaitAny, osWaitForever);
}

/* USER CODE END 1 */
