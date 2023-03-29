/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Variables */
osThreadId Debug_TaskHandle;
osThreadId Chassis_TaskHandle;
osThreadId Data_Tx_TaskHandle;
osThreadId Referee_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Debug_Task_Start(void const * argument);
void Chassis_Task_Start(void const * argument);
void Data_Tx_Task_Start(void const * argument);
void Referee_Task_Start(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of Debug_Task */
  osThreadDef(Debug_Task, Debug_Task_Start, osPriorityIdle, 0, 128);
  Debug_TaskHandle = osThreadCreate(osThread(Debug_Task), NULL);

  /* definition and creation of Chassis_Task */
  osThreadDef(Chassis_Task, Chassis_Task_Start, osPriorityNormal, 0, 1024);
  Chassis_TaskHandle = osThreadCreate(osThread(Chassis_Task), NULL);

  /* definition and creation of Data_Tx_Task */
  osThreadDef(Data_Tx_Task, Data_Tx_Task_Start, osPriorityAboveNormal, 0, 512);
  Data_Tx_TaskHandle = osThreadCreate(osThread(Data_Tx_Task), NULL);

  /* definition and creation of Referee_Task */
  osThreadDef(Referee_Task, Referee_Task_Start, osPriorityNormal, 0, 258);
  Referee_TaskHandle = osThreadCreate(osThread(Referee_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Debug_Task_Start */
/**
  * @brief  Function implementing the Debug_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Debug_Task_Start */
__weak void Debug_Task_Start(void const * argument)
{
  /* USER CODE BEGIN Debug_Task_Start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Debug_Task_Start */
}

/* USER CODE BEGIN Header_Chassis_Task_Start */
/**
* @brief Function implementing the Chassis_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task_Start */
__weak void Chassis_Task_Start(void const * argument)
{
  /* USER CODE BEGIN Chassis_Task_Start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_Task_Start */
}

/* USER CODE BEGIN Header_Data_Tx_Task_Start */
/**
* @brief Function implementing the Data_Tx_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Data_Tx_Task_Start */
__weak void Data_Tx_Task_Start(void const * argument)
{
  /* USER CODE BEGIN Data_Tx_Task_Start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Data_Tx_Task_Start */
}

/* USER CODE BEGIN Header_Referee_Task_Start */
/**
* @brief Function implementing the Referee_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Referee_Task_Start */
__weak void Referee_Task_Start(void const * argument)
{
  /* USER CODE BEGIN Referee_Task_Start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Referee_Task_Start */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
