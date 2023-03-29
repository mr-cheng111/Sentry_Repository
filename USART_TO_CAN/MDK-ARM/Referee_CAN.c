#include "Referee_CAN.h"
#include "bsp_can.h"
#include "bsp_Referee.h"
#include "Referee_Behaviour.h"



//������������Ϣ
robot_information_t robot_information;

uint8_t Referee_Data[8] = {0};

void Get_Robot_Information(void)
{
	robot_information.robot_id = robot_state.robot_id;
	robot_information.power_output = (uint8_t)(((0x01 & robot_state.mains_power_shooter_output) << 2)|
																((0x01 & robot_state.mains_power_chassis_output) << 2) 				 |
																((0x01 & robot_state.mains_power_gimbal_output) << 2));
	robot_information.remain_HP = robot_state.remain_HP;
	robot_information.max_HP = robot_state.max_HP;
}


void Referee_Send_Data(void)
{
	//��ȡ������������Ϣ
	Get_Robot_Information();
	//���ͻ�����������Ϣ
	Can_Send_Data(&hcan,0x129, 6, (uint8_t *) &robot_information);
	HAL_Delay(1);
	//���ݻ�����������Ϣ��ѡ��ͬ�������д���
	switch(robot_information.robot_id & 0xFF)
	{		
		case RED_HERO:
		case BLUE_HERO:
		{
			HERO_Referee_Data_Send();
			break;
		}
		case RED_ENGINEER:
		case BLUE_ENGINEER:
		{
			ENGINEER_Referee_Data_Send();
			break;
		}
		case RED_STANDARD_1:
		case RED_STANDARD_2:
		case RED_STANDARD_3:
		case BLUE_STANDARD_1:
		case BLUE_STANDARD_2:
		case BLUE_STANDARD_3:
		{
			Infantry_Referee_Data_Send();
			break;
		}
		case RED_AERIAL:
		case BLUE_AERIAL:
		{
			AERIAL_Referee_Data_Send();
			break;
		}
		case RED_SENTRY:
		case BLUE_SENTRY:
		{
			Sentry_Referee_Data_Send();
			break;
		}
		default :break;
	}
	Can_Send_Data(&hcan,Referee_CAN_ID,8,Referee_Data);
	HAL_Delay(1);
	
}

/*******Ӣ���������*********/
//����״̬��־λ
//bit0 ������ʼ״̬      	0δ��ʼ 1��ʼ
//bit1 �Ƿ�������ģʽ		0δ��ʼ 1��ʼ
//bit2 �����ɨ��		   	0δ��ʼ 1��ʼ
void HERO_Referee_Data_Send(void)
{
//������ʼ
	if(game_state.game_progress == 4)
	{
		Referee_Data[0] |= 1;
	}
	else
	{
		Referee_Data[0] &= ~1;
	}
	//��Ϊ�췽�ڱ�
	if(robot_state.robot_id == 7)
	{
		//�췽ǰ��ս�ѱ����٣���������ģʽ
		if(game_robot_HP_t.red_outpost_HP == 0)
		{
			Referee_Data[0] |= 1<<1;
		}
		else 
		{
			Referee_Data[0] &= ~(1<<1);
		}
	}
	
	//bit1����λװ�װ��˺�
	Referee_Data[2] |= (robot_hurt_t.armor_type & 0xF)<<4;
	
	//bit3���̻�������
	Referee_Data[3]  = (uint8_t)power_heat_data_t.chassis_power_buffer;
	
	Referee_Data[4] |= (uint8_t)(power_heat_data_t.shooter_id1_17mm_cooling_heat<<8);
	Referee_Data[5] |= (uint8_t)(power_heat_data_t.shooter_id1_17mm_cooling_heat);
	
	Referee_Data[6] |= (uint8_t)(power_heat_data_t.shooter_id2_17mm_cooling_heat<<8);
	Referee_Data[7] |= (uint8_t)(power_heat_data_t.shooter_id2_17mm_cooling_heat);

}
/*******�����������*********/
//����״̬��־λ
//bit0 ������ʼ״̬      	0δ��ʼ 1��ʼ
//bit1 �Ƿ�������ģʽ		0δ��ʼ 1��ʼ
//bit2 �����ɨ��		   	0δ��ʼ 1��ʼ
void ENGINEER_Referee_Data_Send(void)
{
//������ʼ
	if(game_state.game_progress == 4)
	{
		Referee_Data[0] |= 1;
	}
	else
	{
		Referee_Data[0] &= ~1;
	}
	//��Ϊ�췽�ڱ�
	if(robot_state.robot_id == 7)
	{
		//�췽ǰ��ս�ѱ����٣���������ģʽ
		if(game_robot_HP_t.red_outpost_HP == 0)
		{
			Referee_Data[0] |= 1<<1;
		}
		else 
		{
			Referee_Data[0] &= ~(1<<1);
		}
	}
	
	//bit1����λװ�װ��˺�
	Referee_Data[2] |= (robot_hurt_t.armor_type & 0xF)<<4;
	
	//bit3���̻�������
	Referee_Data[3]  = (uint8_t)power_heat_data_t.chassis_power_buffer;
	
	Referee_Data[4] |= (uint8_t)(power_heat_data_t.shooter_id1_17mm_cooling_heat<<8);
	Referee_Data[5] |= (uint8_t)(power_heat_data_t.shooter_id1_17mm_cooling_heat);
	
	Referee_Data[6] |= (uint8_t)(power_heat_data_t.shooter_id2_17mm_cooling_heat<<8);
	Referee_Data[7] |= (uint8_t)(power_heat_data_t.shooter_id2_17mm_cooling_heat);

}
/*******�����������*********/
//����״̬��־λ
//bit0 ������ʼ״̬      	0δ��ʼ 1��ʼ
//bit1 �Ƿ�������ģʽ		0δ��ʼ 1��ʼ
//bit2 �����ɨ��		   	0δ��ʼ 1��ʼ
void Infantry_Referee_Data_Send(void)
{
//������ʼ
	if(game_state.game_progress == 4)
	{
		Referee_Data[0] |= 1;
	}
	else
	{
		Referee_Data[0] &= ~1;
	}
	
	//bit1����λװ�װ��˺�
	Referee_Data[1] |= (robot_hurt_t.armor_type & 0xF)<<4;
	
	//bit2���̻�������
	Referee_Data[2]  = (uint8_t)power_heat_data_t.chassis_power_buffer;
	
	Referee_Data[3] |= (uint8_t)(power_heat_data_t.shooter_id1_17mm_cooling_heat<<8);
	Referee_Data[4] |= (uint8_t)(power_heat_data_t.shooter_id1_17mm_cooling_heat);
	
	Referee_Data[5] |= (uint8_t)(power_heat_data_t.shooter_id2_17mm_cooling_heat<<8);
	Referee_Data[6] |= (uint8_t)(power_heat_data_t.shooter_id2_17mm_cooling_heat);

}
/*******���˻��������*********/
//����״̬��־λ
//bit0 ������ʼ״̬      	0δ��ʼ 1��ʼ
//bit1 �Ƿ�������ģʽ		0δ��ʼ 1��ʼ
//bit2 �����ɨ��		   	0δ��ʼ 1��ʼ
void AERIAL_Referee_Data_Send(void)
{
//������ʼ
	if(game_state.game_progress == 4)
	{
		Referee_Data[0] |= 1;
	}
	else
	{
		Referee_Data[0] &= ~1;
	}

	//��ǰʣ���ӵ�������
	if(bullet_remaining_t.bullet_remaining_num_17mm < 200)
	{
		Referee_Data[0] |= 1<<3;
	}
	else
	{
		Referee_Data[0] &=~(1<<3);
	}
	
	//bit1����λװ�װ��˺�
	Referee_Data[1] |= (robot_hurt_t.armor_type & 0xF)<<4;
	
	//bit2���̻�������
	Referee_Data[2]  = (uint8_t)power_heat_data_t.chassis_power_buffer;
	
	Referee_Data[3] |= (uint8_t)(power_heat_data_t.shooter_id1_17mm_cooling_heat<<8);
	Referee_Data[4] |= (uint8_t)(power_heat_data_t.shooter_id1_17mm_cooling_heat);
	
	Referee_Data[5] |= (uint8_t)(power_heat_data_t.shooter_id2_17mm_cooling_heat<<8);
	Referee_Data[6] |= (uint8_t)(power_heat_data_t.shooter_id2_17mm_cooling_heat);

}

/*******�ڱ��������*********/
//����״̬��־λ
//bit0 ������ʼ״̬      	0δ��ʼ 1��ʼ
//bit1 �Ƿ�������ģʽ		0δ��ʼ 1��ʼ
//bit2 �����ɨ��		   	0δ��ʼ 1��ʼ
void Sentry_Referee_Data_Send(void)
{
//������ʼ
	if(game_state.game_progress == 4)
	{
		Referee_Data[0] |= 1;
	}
	else
	{
		Referee_Data[0] &= ~1;
	}
	//��Ϊ�췽�ڱ�
	if(robot_state.robot_id == 0)
	{
		//�췽ǰ��ս�ѱ����٣���������ģʽ
		if(game_robot_HP_t.red_outpost_HP == 0)
		{
			Referee_Data[0] |= 1<<1;
		}
		else 
		{
			Referee_Data[0] &= ~(1<<1);
		}
	}
	//��Ϊ�����ڱ�
	else if(robot_state.robot_id == 107)
	{
		//����ǰ��ս�ѱ����٣���������ģʽ
		if(game_robot_HP_t.blue_outpost_HP == 0)
		{
			Referee_Data[0] |= 1<<1;
		}
		else 
		{
			Referee_Data[0] &=~(1<<1);
		}
	}
	//bit0����λΪװ�װ�����
	Referee_Data[0] |= (robot_hurt_t.armor_type & 0xF)<<4;
	//��ǰʣ���ӵ�������
	Referee_Data[1] = bullet_remaining_t.bullet_remaining_num_17mm<<8;
	Referee_Data[2] = bullet_remaining_t.bullet_remaining_num_17mm;
	//bit3���̻�������
	Referee_Data[3]  = (uint8_t)power_heat_data_t.chassis_power_buffer;
	
	Referee_Data[4] |= (uint8_t)(power_heat_data_t.shooter_id1_17mm_cooling_heat<<8);
	Referee_Data[5] |= (uint8_t)(power_heat_data_t.shooter_id1_17mm_cooling_heat);
	
	Referee_Data[6] |= (uint8_t)(power_heat_data_t.shooter_id2_17mm_cooling_heat<<8);
	Referee_Data[7] |= (uint8_t)(power_heat_data_t.shooter_id2_17mm_cooling_heat);

}

