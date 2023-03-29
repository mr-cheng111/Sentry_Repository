#include "Referee_CAN.h"
#include "bsp_can.h"
#include "bsp_Referee.h"
#include "Referee_Behaviour.h"



//机器人自身信息
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
	//获取机器人自身信息
	Get_Robot_Information();
	//发送机器人自身信息
	Can_Send_Data(&hcan,0x129, 6, (uint8_t *) &robot_information);
	HAL_Delay(1);
	//根据机器人自身信息，选择不同函数进行处理
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

/*******英雄所需参数*********/
//比赛状态标志位
//bit0 比赛开始状态      	0未开始 1开始
//bit1 是否开启保卫模式		0未开始 1开始
//bit2 点射或扫射		   	0未开始 1开始
void HERO_Referee_Data_Send(void)
{
//比赛开始
	if(game_state.game_progress == 4)
	{
		Referee_Data[0] |= 1;
	}
	else
	{
		Referee_Data[0] &= ~1;
	}
	//若为红方哨兵
	if(robot_state.robot_id == 7)
	{
		//红方前哨战已被击毁，开启保卫模式
		if(game_robot_HP_t.red_outpost_HP == 0)
		{
			Referee_Data[0] |= 1<<1;
		}
		else 
		{
			Referee_Data[0] &= ~(1<<1);
		}
	}
	
	//bit1高四位装甲板伤害
	Referee_Data[2] |= (robot_hurt_t.armor_type & 0xF)<<4;
	
	//bit3底盘缓冲能量
	Referee_Data[3]  = (uint8_t)power_heat_data_t.chassis_power_buffer;
	
	Referee_Data[4] |= (uint8_t)(power_heat_data_t.shooter_id1_17mm_cooling_heat<<8);
	Referee_Data[5] |= (uint8_t)(power_heat_data_t.shooter_id1_17mm_cooling_heat);
	
	Referee_Data[6] |= (uint8_t)(power_heat_data_t.shooter_id2_17mm_cooling_heat<<8);
	Referee_Data[7] |= (uint8_t)(power_heat_data_t.shooter_id2_17mm_cooling_heat);

}
/*******工程所需参数*********/
//比赛状态标志位
//bit0 比赛开始状态      	0未开始 1开始
//bit1 是否开启保卫模式		0未开始 1开始
//bit2 点射或扫射		   	0未开始 1开始
void ENGINEER_Referee_Data_Send(void)
{
//比赛开始
	if(game_state.game_progress == 4)
	{
		Referee_Data[0] |= 1;
	}
	else
	{
		Referee_Data[0] &= ~1;
	}
	//若为红方哨兵
	if(robot_state.robot_id == 7)
	{
		//红方前哨战已被击毁，开启保卫模式
		if(game_robot_HP_t.red_outpost_HP == 0)
		{
			Referee_Data[0] |= 1<<1;
		}
		else 
		{
			Referee_Data[0] &= ~(1<<1);
		}
	}
	
	//bit1高四位装甲板伤害
	Referee_Data[2] |= (robot_hurt_t.armor_type & 0xF)<<4;
	
	//bit3底盘缓冲能量
	Referee_Data[3]  = (uint8_t)power_heat_data_t.chassis_power_buffer;
	
	Referee_Data[4] |= (uint8_t)(power_heat_data_t.shooter_id1_17mm_cooling_heat<<8);
	Referee_Data[5] |= (uint8_t)(power_heat_data_t.shooter_id1_17mm_cooling_heat);
	
	Referee_Data[6] |= (uint8_t)(power_heat_data_t.shooter_id2_17mm_cooling_heat<<8);
	Referee_Data[7] |= (uint8_t)(power_heat_data_t.shooter_id2_17mm_cooling_heat);

}
/*******步兵所需参数*********/
//比赛状态标志位
//bit0 比赛开始状态      	0未开始 1开始
//bit1 是否开启保卫模式		0未开始 1开始
//bit2 点射或扫射		   	0未开始 1开始
void Infantry_Referee_Data_Send(void)
{
//比赛开始
	if(game_state.game_progress == 4)
	{
		Referee_Data[0] |= 1;
	}
	else
	{
		Referee_Data[0] &= ~1;
	}
	
	//bit1高四位装甲板伤害
	Referee_Data[1] |= (robot_hurt_t.armor_type & 0xF)<<4;
	
	//bit2底盘缓冲能量
	Referee_Data[2]  = (uint8_t)power_heat_data_t.chassis_power_buffer;
	
	Referee_Data[3] |= (uint8_t)(power_heat_data_t.shooter_id1_17mm_cooling_heat<<8);
	Referee_Data[4] |= (uint8_t)(power_heat_data_t.shooter_id1_17mm_cooling_heat);
	
	Referee_Data[5] |= (uint8_t)(power_heat_data_t.shooter_id2_17mm_cooling_heat<<8);
	Referee_Data[6] |= (uint8_t)(power_heat_data_t.shooter_id2_17mm_cooling_heat);

}
/*******无人机所需参数*********/
//比赛状态标志位
//bit0 比赛开始状态      	0未开始 1开始
//bit1 是否开启保卫模式		0未开始 1开始
//bit2 点射或扫射		   	0未开始 1开始
void AERIAL_Referee_Data_Send(void)
{
//比赛开始
	if(game_state.game_progress == 4)
	{
		Referee_Data[0] |= 1;
	}
	else
	{
		Referee_Data[0] &= ~1;
	}

	//当前剩余子弹发弹量
	if(bullet_remaining_t.bullet_remaining_num_17mm < 200)
	{
		Referee_Data[0] |= 1<<3;
	}
	else
	{
		Referee_Data[0] &=~(1<<3);
	}
	
	//bit1高四位装甲板伤害
	Referee_Data[1] |= (robot_hurt_t.armor_type & 0xF)<<4;
	
	//bit2底盘缓冲能量
	Referee_Data[2]  = (uint8_t)power_heat_data_t.chassis_power_buffer;
	
	Referee_Data[3] |= (uint8_t)(power_heat_data_t.shooter_id1_17mm_cooling_heat<<8);
	Referee_Data[4] |= (uint8_t)(power_heat_data_t.shooter_id1_17mm_cooling_heat);
	
	Referee_Data[5] |= (uint8_t)(power_heat_data_t.shooter_id2_17mm_cooling_heat<<8);
	Referee_Data[6] |= (uint8_t)(power_heat_data_t.shooter_id2_17mm_cooling_heat);

}

/*******哨兵所需参数*********/
//比赛状态标志位
//bit0 比赛开始状态      	0未开始 1开始
//bit1 是否开启保卫模式		0未开始 1开始
//bit2 点射或扫射		   	0未开始 1开始
void Sentry_Referee_Data_Send(void)
{
//比赛开始
	if(game_state.game_progress == 4)
	{
		Referee_Data[0] |= 1;
	}
	else
	{
		Referee_Data[0] &= ~1;
	}
	//若为红方哨兵
	if(robot_state.robot_id == 0)
	{
		//红方前哨战已被击毁，开启保卫模式
		if(game_robot_HP_t.red_outpost_HP == 0)
		{
			Referee_Data[0] |= 1<<1;
		}
		else 
		{
			Referee_Data[0] &= ~(1<<1);
		}
	}
	//若为蓝方哨兵
	else if(robot_state.robot_id == 107)
	{
		//蓝方前哨战已被击毁，开启保卫模式
		if(game_robot_HP_t.blue_outpost_HP == 0)
		{
			Referee_Data[0] |= 1<<1;
		}
		else 
		{
			Referee_Data[0] &=~(1<<1);
		}
	}
	//bit0高四位为装甲板类型
	Referee_Data[0] |= (robot_hurt_t.armor_type & 0xF)<<4;
	//当前剩余子弹发弹量
	Referee_Data[1] = bullet_remaining_t.bullet_remaining_num_17mm<<8;
	Referee_Data[2] = bullet_remaining_t.bullet_remaining_num_17mm;
	//bit3底盘缓冲能量
	Referee_Data[3]  = (uint8_t)power_heat_data_t.chassis_power_buffer;
	
	Referee_Data[4] |= (uint8_t)(power_heat_data_t.shooter_id1_17mm_cooling_heat<<8);
	Referee_Data[5] |= (uint8_t)(power_heat_data_t.shooter_id1_17mm_cooling_heat);
	
	Referee_Data[6] |= (uint8_t)(power_heat_data_t.shooter_id2_17mm_cooling_heat<<8);
	Referee_Data[7] |= (uint8_t)(power_heat_data_t.shooter_id2_17mm_cooling_heat);

}

