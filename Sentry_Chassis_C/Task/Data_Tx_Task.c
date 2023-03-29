#include "Data_Tx_Task.h"
#include "Chassis_Task.h"

#include "cmsis_os.h"
#include "can.h"
#include "gpio.h"
#include "usart.h"
#include "stdio.h"
#include "Referee_Behaviour.h"
#include "ChassisPowerBehaviour.h"
#include "IT_Sever.h"

#include "bsp_can.h"
#include "bsp_led.h"



extern void Can_Send_Data(CAN_HandleTypeDef *hcan,uint16_t frame,uint8_t *data);
extern Chassis_Typedef Chassis;



uint8_t DataBuffer[11];
uint8_t RC_Data[8];
uint8_t Chassis_Mode_Data[8];


static CAN_TxHeaderTypeDef can_tx_message;

static void Can_Send(CAN_HandleTypeDef *hcan,uint32_t id,uint8_t lenth,uint8_t *buffer);

extern ext_power_heat_data_t power_heat_data_t;

#define printf(...)			HAL_UART_Transmit_DMA(&huart1,\
                                         (uint8_t *)u1_buf,\
                                         sprintf((char*)u1_buf,__VA_ARGS__))			
uint8_t u1_buf[20];
fp32 Calc_Power = 0;

extern robot_information_t robot_information;
                                         
uint8_t send_flag = 0;
                                         
void Data_Tx_Task_Start(void const * argument)
{
	int flag = 0;
	uint8_t SendLenth;
	uint32_t SendId;
  for(;;)
  {
     CAN1_CMD(Chassis.Chassis_Control_Data.Power_Motor_Current,0,Chassis.Chassis_Control_Data.Brake_Motor_Current,0,0,0,0,0);
//     CAN1_CMD(0,0,0,0,0,0,0,0);

	 Get_Remote_Data(Chassis.Chassis_RC);
	 Get_Chassis_Data(Chassis.Chassis_Mode_Flag.Current_Control_Mode);
	 
	
//	  Can_Send_Data(&hcan2,RC_DATA_ID,RC_Data);
	 
	  //use to send the RC Ctrl Data
	if (send_flag == 0) 
	{
		send_flag = 1;
		Can_Send_Data(&hcan2,RC_DATA_ID,RC_Data);
	}
	//Send the robot information
	else if (send_flag == 1) 
	{
		send_flag = 2;
		Can_Send(&hcan2, 0x129, 6, (uint8_t *) &robot_information);
	}
	//Send the Bullet Speed
	else if (send_flag == 2) 
	{
		send_flag = 0;
		fp32 shoot_speed = (shoot_data_t.bullet_speed);
		uint8_t a = sizeof(ext_shoot_data_t);
		Can_Send(&hcan2, 0x122,4,(uint8_t *)&shoot_speed);
	}
//	 Can_Send_Data(&hcan1,ROBOT_DATA_ID,
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
void Get_Remote_Data(const RC_ctrl_t *rc_data)
{
	RC_Data[0] = rc_data->rc.ch[2]>>8;
	RC_Data[1] = rc_data->rc.ch[2]; 
	//rc_left_channel_y
	RC_Data[2] = rc_data->rc.ch[3]>>8;
	RC_Data[3] = rc_data->rc.ch[3];
	//rc_up_channel
	RC_Data[4] = rc_data->rc.ch[4]>>8;
	RC_Data[5] = rc_data->rc.ch[4];
	RC_Data[6] = rc_data->rc.s[1]<<4 | rc_data->rc.s[0];
	RC_Data[7] = 0;
}

void Get_Chassis_Data(uint8_t Chassis_Mode)
{
	//����״̬��־λ
	//bit0 ������ʼ״̬        0δ��ʼ 1��ʼ
	//bit1 �Ƿ�������ģʽ    0δ��ʼ 1��ʼ
	//bit2 �����ɨ��		
	//bit3 ʣ���ӵ�������
	//bit4~7 ״̬��־λ
	if(game_state.game_progress == 4)
	{
		RC_Data[7] |= 1;
	}
	else
	{
		RC_Data[7] &= ~1;
	}
	//��Ϊ�췽�ڱ�
	if(robot_state.robot_id == 7)
	{
		//�췽ǰ��ս�ѱ����٣���������ģʽ
		if(game_robot_HP_t.red_outpost_HP == 0)
		{
			RC_Data[7] |= 1<<1;
		}
		else 
		{
			RC_Data[7] &= ~(1<<1);
		}
	}
	//��Ϊ�����ڱ�
	else if(robot_state.robot_id == 107)
	{
		//����ǰ��ս�ѱ����٣���������ģʽ
		if(game_robot_HP_t.blue_outpost_HP == 0)
		{
			RC_Data[7] |= 1<<1;
		}
		else 
		{
			RC_Data[7] &=~(1<<1);
		}
	}
	//��ǰʣ���ӵ�������
	if(bullet_remaining_t.bullet_remaining_num_17mm < 200)
	{
		RC_Data[7] |= 1<<3;
	}
	else
	{
		RC_Data[7] &=~(1<<3);
	}
	switch(Chassis.Chassis_Mode_Flag.Current_Control_Mode)
	{
		case CHASSIS_WEEK :
		{
			RC_Data[7] |= CHASSIS_WEEK<<4;
			break;
		}
		case CHASSIS_STOP :
		{
			RC_Data[7] |= CHASSIS_STOP<<4;
			break;
		}
		case CHASSIS_FAST :
		{
			RC_Data[7] |= CHASSIS_FAST<<4;
			break;
		}
		case CHASSIS_BRAKE:
		{
			RC_Data[7] |= CHASSIS_BRAKE<<4;
			break;
		}
		default :
		{
			RC_Data[7] |= CHASSIS_WEEK<<4;
			break;
		}

	}
	
}



