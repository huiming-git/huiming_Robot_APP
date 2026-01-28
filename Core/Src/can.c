/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
/* Minimal CAN Rx queue (ISR -> task). */
#define APP_CAN_RXQ_CAPACITY 64U
static volatile uint32_t s_rxq_w = 0;
static volatile uint32_t s_rxq_r = 0;
static App_CanFrame_t s_rxq[APP_CAN_RXQ_CAPACITY];

static uint32_t rxq_next(uint32_t idx) { return (idx + 1U) % APP_CAN_RXQ_CAPACITY; }

volatile uint32_t g_app_can1_rx_dropped = 0;
volatile uint32_t g_app_can2_rx_dropped = 0;
volatile uint32_t g_app_can1_tx_mailbox_full = 0;
volatile uint32_t g_app_can2_tx_mailbox_full = 0;
volatile uint32_t g_app_can1_tx_add_fail = 0;
volatile uint32_t g_app_can2_tx_add_fail = 0;

/* CAN Tx completion tracking for Operation model (BLOCK/CALLBACK). */
#include "FreeRTOS.h"
#include "task.h"

typedef struct
{
  TaskHandle_t waiting_task;
  App_OpCallback cb;
  void* user;
  volatile uint8_t active;
  volatile uint8_t ok;
} app_can_tx_slot_t;

static app_can_tx_slot_t s_tx1[3] = {0};
static app_can_tx_slot_t s_tx2[3] = {0};

static app_can_tx_slot_t* tx_slots_for(CAN_HandleTypeDef* hcan)
{
  return (hcan->Instance == CAN2) ? s_tx2 : s_tx1;
}

static int mailbox_index(uint32_t mailbox)
{
  switch (mailbox)
  {
    case CAN_TX_MAILBOX0:
      return 0;
    case CAN_TX_MAILBOX1:
      return 1;
    case CAN_TX_MAILBOX2:
      return 2;
    default:
      return -1;
  }
}

static void tx_complete_from_isr(CAN_HandleTypeDef* hcan, uint32_t mailbox, uint8_t ok)
{
  const int idx = mailbox_index(mailbox);
  if (idx < 0) return;

  app_can_tx_slot_t* slots = tx_slots_for(hcan);
  app_can_tx_slot_t* s = &slots[idx];
  if (!s->active) return;

  s->ok = ok ? 1U : 0U;
  s->active = 0U;

  if (s->cb != NULL)
  {
    s->cb(s->ok, 1U, s->user);
  }

  BaseType_t hpw = pdFALSE;
  if (s->waiting_task != NULL)
  {
    vTaskNotifyGiveFromISR(s->waiting_task, &hpw);
  }
  portYIELD_FROM_ISR(hpw);
}
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef filter = {0};
  filter.FilterActivation = ENABLE;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.SlaveStartFilterBank = 14;

  filter.FilterBank = 0;
  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
  {
    Error_Handler();
  }
  filter.FilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
  {
    Error_Handler();
  }

  (void)HAL_CAN_Start(&hcan1);
  (void)HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);
  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  (void)HAL_CAN_Start(&hcan2);
  (void)HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);
  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5|GPIO_PIN_6);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
  {
    return;
  }

  const uint32_t w = s_rxq_w;
  const uint32_t next = rxq_next(w);
  if (next == s_rxq_r)
  {
    if (hcan->Instance == CAN2)
      g_app_can2_rx_dropped += 1U;
    else
      g_app_can1_rx_dropped += 1U;
    return;
  }

  App_CanFrame_t f;
  f.bus = (hcan->Instance == CAN2) ? 2U : 1U;
  f.std_id = (uint16_t)(rx_header.StdId & 0x7FFU);
  f.dlc = (uint8_t)rx_header.DLC;
  for (uint8_t i = 0; i < 8; i++)
  {
    f.data[i] = rx_data[i];
  }

  s_rxq[w] = f;
  s_rxq_w = next;
}

uint8_t App_Can_Tx(uint8_t bus, uint16_t std_id, const uint8_t *data, uint8_t dlc)
{
  return App_Can_TxOp(bus, std_id, data, dlc, NULL);
}

uint8_t App_Can_TxOp(uint8_t bus, uint16_t std_id, const uint8_t *data, uint8_t dlc,
                     const App_Operation* op)
{
  if (dlc > 8U) return 0U;

  CAN_HandleTypeDef *h = (bus == 2U) ? &hcan2 : &hcan1;
  if (HAL_CAN_GetTxMailboxesFreeLevel(h) == 0U)
  {
    if (bus == 2U)
      g_app_can2_tx_mailbox_full += 1U;
    else
      g_app_can1_tx_mailbox_full += 1U;
    return 0U;
  }

  CAN_TxHeaderTypeDef header = {0};
  header.StdId = std_id & 0x7FFU;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = dlc;
  header.TransmitGlobalTime = DISABLE;

  uint8_t payload[8] = {0};
  if (data != NULL)
  {
    for (uint8_t i = 0; i < dlc; i++)
    {

      payload[i] = data[i];
    }
  }

  uint32_t mailbox = 0;
  if (HAL_CAN_AddTxMessage(h, &header, payload, &mailbox) != HAL_OK)
  {
    if (bus == 2U)
      g_app_can2_tx_add_fail += 1U;
    else
      g_app_can1_tx_add_fail += 1U;
    return 0U;
  }

  App_Operation local = App_OpNone();
  if (op != NULL) local = *op;

  if (local.mode == APP_OP_BLOCK || local.mode == APP_OP_CALLBACK)
  {
    const int idx = mailbox_index(mailbox);
    if (idx < 0) return 1U;

    app_can_tx_slot_t* slots = tx_slots_for(h);
    slots[idx].ok = 0U;
    slots[idx].active = 1U;
    slots[idx].waiting_task = NULL;
    slots[idx].cb = NULL;
    slots[idx].user = NULL;

    if (local.mode == APP_OP_BLOCK)
    {
      slots[idx].waiting_task = xTaskGetCurrentTaskHandle();
      const uint32_t timeout_ms = (local.timeout_ms == 0U) ? 5U : local.timeout_ms;
      const uint32_t taken = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms));
      if (taken == 0U)
      {
        (void)HAL_CAN_AbortTxRequest(h, mailbox);
        slots[idx].active = 0U;
        return 0U;
      }
      return (slots[idx].ok != 0U) ? 1U : 0U;
    }
    else
    {
      slots[idx].cb = local.cb;
      slots[idx].user = local.user;
    }
  }

  return 1U;
}

uint8_t App_Can_RxPop(App_CanFrame_t *out_frame)
{
  if (out_frame == NULL) return 0U;

  const uint32_t r = s_rxq_r;
  if (r == s_rxq_w) return 0U;

  *out_frame = s_rxq[r];
  s_rxq_r = rxq_next(r);
  return 1U;
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* hcan)
{
  tx_complete_from_isr(hcan, CAN_TX_MAILBOX0, 1U);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* hcan)
{
  tx_complete_from_isr(hcan, CAN_TX_MAILBOX1, 1U);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef* hcan)
{
  tx_complete_from_isr(hcan, CAN_TX_MAILBOX2, 1U);
}

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef* hcan)
{
  tx_complete_from_isr(hcan, CAN_TX_MAILBOX0, 0U);
}

void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef* hcan)
{
  tx_complete_from_isr(hcan, CAN_TX_MAILBOX1, 0U);
}

void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef* hcan)
{
  tx_complete_from_isr(hcan, CAN_TX_MAILBOX2, 0U);
}
/* USER CODE END 1 */
