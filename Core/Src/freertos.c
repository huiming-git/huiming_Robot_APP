/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_bridge.h"
#include "app_can.h"
#include "app_params.h"
#include "can.h"
#include "usart.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

volatile uint64_t g_control_last_ts_us;
volatile uint32_t g_control_dt_us;
volatile int32_t g_control_jitter_us;

osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t sbusTaskHandle;
const osThreadAttr_t sbusTask_attributes = {
  .name = "sbusTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t logicTaskHandle;
const osThreadAttr_t logicTask_attributes = {
  .name = "logicTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t canDebugTaskHandle;
const osThreadAttr_t canDebugTask_attributes = {
  .name = "canDebugTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityIdle,
};

volatile uint8_t g_app_can2_dbg_enable = 1;
volatile uint16_t g_app_can2_dbg_std_id = 0x04;
volatile uint16_t g_app_can2_dbg_ids[4] = {1U, 2U, 3U, 4U};
volatile uint8_t g_app_can2_dbg_dlc = 8;
volatile uint8_t g_app_can2_dbg_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
volatile uint32_t g_app_can2_dbg_period_ms = 100;
volatile uint32_t g_app_can2_dbg_sent = 0;
volatile uint32_t g_app_can2_dbg_failed = 0;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartControlTask(void *argument);
void StartSensorTask(void *argument);
void StartSbusTask(void *argument);
void StartLogicTask(void *argument);
void StartCanDebugTask(void *argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  controlTaskHandle = osThreadNew(StartControlTask, NULL, &controlTask_attributes);
  sensorTaskHandle = osThreadNew(StartSensorTask, NULL, &sensorTask_attributes);
  sbusTaskHandle = osThreadNew(StartSbusTask, NULL, &sbusTask_attributes);
  logicTaskHandle = osThreadNew(StartLogicTask, NULL, &logicTask_attributes);
  canDebugTaskHandle = osThreadNew(StartCanDebugTask, NULL, &canDebugTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    App_Params_Service();
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartControlTask(void *argument)
{
  (void)argument;

  App_Tim1_SchedulerStart();
  RobotApp_Init();

  uint64_t last_ts = 0;
  for (;;)
  {
    (void)osThreadFlagsWait(APP_CONTROL_TICK_FLAG, osFlagsWaitAny, osWaitForever);
    g_control_last_ts_us = App_Tim1_LastTimestampUs();
    if (last_ts != 0) {
      g_control_dt_us = (uint32_t)(g_control_last_ts_us - last_ts);
      g_control_jitter_us = (int32_t)g_control_dt_us - 1000;
    }
    last_ts = g_control_last_ts_us;
    RobotApp_ControlTick(g_control_last_ts_us);
  }
}

void StartCanDebugTask(void *argument)
{
  (void)argument;
  for (;;)
  {
    if (g_app_can2_dbg_enable != 0U)
    {
      static const uint8_t enable_data[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                                             0xFF, 0xFF, 0xFF, 0xFC};
      static uint8_t idx = 0U;

      uint16_t std_id = 0U;
      for (uint8_t k = 0; k < 4U; ++k)
      {
        const uint8_t probe = (uint8_t)((idx + k) & 0x03U);
        const uint16_t cand = g_app_can2_dbg_ids[probe];
        if (cand != 0U)
        {
          std_id = cand;
          idx = (uint8_t)((probe + 1U) & 0x03U);
          break;
        }
      }

      if (std_id != 0U)
      {
        g_robotapp_dm_enable_tx_last_std_id = std_id;
        if (std_id >= 1U && std_id <= 4U)
        {
          const uint8_t m = (uint8_t)(std_id - 1U);
          g_robotapp_dm_enable_tx_attempt[m] += 1U;
          if (App_Can_Tx(2U, std_id, enable_data, 8U) != 0U)
          {
            g_robotapp_dm_enable_tx_started[m] += 1U;
            g_app_can2_dbg_sent += 1U;
          }
          else
          {
            g_app_can2_dbg_failed += 1U;
          }
        }
        else
        {
          if (App_Can_Tx(2U, std_id, enable_data, 8U) != 0U)
            g_app_can2_dbg_sent += 1U;
          else
            g_app_can2_dbg_failed += 1U;
        }
      }
    }

    uint32_t period_ms = g_app_can2_dbg_period_ms;
    if (period_ms == 0U) period_ms = 1U;
    osDelay(period_ms);
  }
}

/* StartLogicTask is implemented in C++ (RobotApp/Tasks/logic_task.cpp). */
/* StartSbusTask is implemented in C++ (RobotApp/Tasks/sbus_task.cpp). */

/* USER CODE END Application */

