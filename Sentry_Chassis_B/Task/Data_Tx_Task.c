#include "Data_Tx_Task.h"
#include "cmsis_os.h"
#include "can.h"
#include "bsp_can.h"
#include "Referee_Behaviour.h"
#include "gpio.h"
#include "usart.h"
#include "stdio.h"
#include "ChassisPowerBehaviour.h"
#include "Chassis_Task.h"

void Can_Send_RC_Data(CAN_HandleTypeDef *hcan,RC_ctrl_t *rc_data);
extern Chassis_Typedef Chassis;



uint8_t DataBuffer[11];
extern osThreadId Data_Tx_TaskHandle;
extern osThreadId Referee_TaskHandle;

static CAN_TxHeaderTypeDef can_tx_message;

static void Can_Send(CAN_HandleTypeDef *hcan,uint32_t id,uint8_t lenth,uint8_t *buffer);



extern ext_power_heat_data_t power_heat_data_t;

#define printf(...)			HAL_UART_Transmit_DMA(&huart3,\
                                         (uint8_t *)u1_buf,\
                                         sprintf((char*)u1_buf,__VA_ARGS__))			
uint8_t u1_buf[20];
fp32 Calc_Power = 0;
void Data_Tx_Task_Start(void const * argument)
{
	int flag = 0;
	uint8_t SendLenth;
	uint32_t SendId;
  for(;;)
  {
		Calc_Power = chassis_powerloop(&Chassis);
		printf("%f,%d\r\n",power_heat_data_t.chassis_power,power_heat_data_t.chassis_power_buffer);
		
		flag = Pop(SendBuffer,(SendBuffer+1),DataBuffer);
		switch(flag)
		{
			case -1:
			{
				ComLedError();//一帧数据不是11的倍数，通信错误
				break;
			}
			case 1:
			{
				SendId = (uint32_t)(DataBuffer[0]<<8|DataBuffer[1]);
				SendLenth = DataBuffer[2];
				Can_Send(&hcan2,SendId,SendLenth,&DataBuffer[3]);
				HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_8);
				break;
			}
			case 0://FIFO空
			{
				vTaskResume(Referee_TaskHandle);
				vTaskSuspend(Data_Tx_TaskHandle);
			}
			default:
			{
				break;
			}
		}
		
		CAN1_CMD(Chassis.Chassis_Control_Data.Power_Motor_Current,Chassis.Chassis_Control_Data.Brake_Motor_Current,0,0,0,0,0,0);
		
		Can_Send_RC_Data(&hcan2,Chassis.Chassis_RC);
		
    osDelay(1);
  }

}


static void Can_Send(CAN_HandleTypeDef *hcan,uint32_t id,uint8_t lenth,uint8_t *buffer)
{
	uint32_t send_mail_box;
    can_tx_message.StdId = id;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = lenth;
	
    HAL_CAN_AddTxMessage(hcan, &can_tx_message, buffer, &send_mail_box);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) 
	{
    HAL_IncTick();
  }
}



/**
	*当mode第一位为1时，控制上云台，当mode第一位为0时，控制下云台
	*当mode最高三位为0时，云台无力
	*当mode最高三位为1时，上云台遥控控制
	*当mode最高三位为2时，下云台遥控控制
	*当mode最高三位为3时，上云台自瞄控制
	*当mode最高三位为4时，下云台自瞄控制
**/
void CAN_Chassis_Data_Packed()
{
	
}



