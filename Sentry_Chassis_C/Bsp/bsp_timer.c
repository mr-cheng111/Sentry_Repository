#include "bsp_timer.h"
#include "tim.h"
#include "math.h"
#include "main.h"



uint32_t rand_num_flag = 0;
uint32_t systerm_time = 0;

void Timer_Start(void)
{
		HAL_TIM_Base_Start_IT(&htim13);
		HAL_TIM_Base_Start_IT(&htim14);
}

/**
	* @brief				10ms��ʱ��
	*/
void TIM8_UP_TIM13_IRQHandler(void)
{
	if(rand_num_flag>0)
	{
		rand_num_flag --;
	}

  HAL_TIM_IRQHandler(&htim13);
}



/**
  * @brief 				1ms��ʱ��
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
	
	systerm_time ++;//��ȡϵͳʱ��
  HAL_TIM_IRQHandler(&htim14);
}

