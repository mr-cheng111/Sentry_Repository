/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "pid.h"
#include "CAN_Receive.h"
#include "Gimbal_task.h"
#include "usart.h"
#include "imu.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Uprintf(...)			HAL_UART_Transmit(&huart1,\
                                           (uint8_t *)buf,\
                                           sprintf((char*)buf,__VA_ARGS__),\
                                           0xffff)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

const motor_measure_t *speed;
uint8_t buf[50];


/* USER CODE END Variables */
osThreadId Gimbal_taskHandle;
osThreadId Detect_TaskHandle;
osThreadId Communicate_TasHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Gimbal_task_Start(void const * argument);
void Detect_Task_Start(void const * argument);
void Communicate_Task_Start(void const * argument);

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
  /* definition and creation of Gimbal_task */
  osThreadDef(Gimbal_task, Gimbal_task_Start, osPriorityNormal, 0, 128);
  Gimbal_taskHandle = osThreadCreate(osThread(Gimbal_task), NULL);

  /* definition and creation of Detect_Task */
  osThreadDef(Detect_Task, Detect_Task_Start, osPriorityNormal, 0, 512);
  Detect_TaskHandle = osThreadCreate(osThread(Detect_Task), NULL);

  /* definition and creation of Communicate_Tas */
  osThreadDef(Communicate_Tas, Communicate_Task_Start, osPriorityNormal, 0, 512);
  Communicate_TasHandle = osThreadCreate(osThread(Communicate_Tas), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Gimbal_task_Start */
///////////////////////////////**
//////////////////////////////  * @brief  Function implementing the Gimbal_task thread.
//////////////////////////////  * @param  argument: Not used
//////////////////////////////  * @retval None
//////////////////////////////  */
/* USER CODE END Header_Gimbal_task_Start */
//void Gimbal_task_Start(void const * argument)
//{
  /* USER CODE BEGIN Gimbal_task_Start */
//////////////////////////////  /* Infinite loop */
//////////////////////////////  for(;;)
//////////////////////////////  {
//////////////////////////////    osDelay(1);
//////////////////////////////  }
  /* USER CODE END Gimbal_task_Start */
//}

/* USER CODE BEGIN Header_Detect_Task_Start */
/**
* @brief Function implementing the Detect_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Detect_Task_Start */
void Detect_Task_Start(void const * argument)
{
  /* USER CODE BEGIN Detect_Task_Start */
  /* Infinite loop */
  for(;;)
  {
//		printf();
				
    osDelay(1);
  }
  /* USER CODE END Detect_Task_Start */
}

/* USER CODE BEGIN Header_Communicate_Task_Start */
/**
* @brief Function implementing the Communicate_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Communicate_Task_Start */
void Communicate_Task_Start(void const * argument)
{
  /* USER CODE BEGIN Communicate_Task_Start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Communicate_Task_Start */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
