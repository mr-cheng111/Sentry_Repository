#include "RefereeCan.h"

#include <string.h>


RefereeInformation_t RefereeInformation;
Robot_Data_t Robot_Data;
Game_State_t Game_State;
void RefereePowerHeatNode0InformationUpdate(uint8_t *data)
{
    memcpy(&RefereeInformation.Realtime.ChassisVoltage, data, 8);
}
void RefereePowerHeatNode1InformationUpdate(uint8_t *data)
{
    memcpy(&RefereeInformation.Realtime.ChassisBufferEnergy, data, 8);
}
void RefereeAmmoSpeedNode0InformationUpdate(uint8_t *data)
{
    memcpy(&RefereeInformation.Ammo0Speed, data, 4);
}
void RefereeAmmoSpeedNode1InformationUpdate(uint8_t *data)
{
    memcpy(&RefereeInformation.Ammo1Speed, data, 4);
}
void RefereeAmmoSpeedNode2InformationUpdate(uint8_t *data)
{
    memcpy(&RefereeInformation.Ammo2Speed, data, 4);
}
void RefereeAmmoLimitNode0InformationUpdate(uint8_t *data)
{
    memcpy(&RefereeInformation.Ammo0Limit.Cooling, data, 6);
}
void RefereeAmmoLimitNode1InformationUpdate(uint8_t *data)
{
    memcpy(&RefereeInformation.Ammo1Limit.Cooling, data, 6);
}
void RefereeAmmoLimitNode2InformationUpdate(uint8_t *data)
{
    memcpy(&RefereeInformation.Ammo2Limit.Cooling, data, 6);
}
void RefereeSelfStateNodeInformationUpdate(uint8_t *data)
{
    memcpy(&RefereeInformation.RobotState.RobotID, data, 6);
}

void GetRefereeInformation(RefereeInformation_t *Inf)
{
    memcpy(Inf, &RefereeInformation, sizeof(RefereeInformation_t));
}

void Get_Referee_Data(uint8_t *rx_data)
{
	switch(rx_data[0]&1)
	{
		case 1:Game_State.Game_Start_State = Game_START;break;
		case 0:Game_State.Game_Start_State = Game_STOP; break;
		default: Game_State.Game_Start_State = Game_STOP;break;
	}
	switch((rx_data[0]&(1<<1))>>1)
	{
		case 1:  Game_State.Defend_Mode = Sentry_Defend;break;
		case 0:  Game_State.Defend_Mode = Sentry_Escape;break;
		default: Game_State.Defend_Mode = Sentry_Defend;break;
	}
	switch((rx_data[0]&(1<<2))>>2)
	{
		case 1:  Game_State.Point_Fire_Mode = Sentry_Point_Fire;break;
		case 0:  Game_State.Point_Fire_Mode = Sentry_Brust_Fire;break;
		default: Game_State.Point_Fire_Mode = Sentry_Brust_Fire;break;
	}
	Robot_Data.Armor_Type |= (rx_data[0]>>4)&0xF;
	
	//当前剩余子弹发弹量
	Robot_Data.bullet_remaining_num_17mm= (rx_data[1]<<8)|rx_data[2];
	//bit2高四位装甲板伤害

	//bit3底盘缓冲能量
	Robot_Data.chassis_power_buffer  = rx_data[3];
	
	Robot_Data.shooter_id1_17mm_cooling_heat = rx_data[4]<<8|rx_data[5];
	Robot_Data.shooter_id2_17mm_cooling_heat = rx_data[6]<<8|rx_data[7];

}


// 热量闭环

void AmmoHeatSettlementInterpolation(void)
{
    // 10Hz Loop
    RefereeInformation.Realtime.Ammo0Heat -= (RefereeInformation.Ammo0Limit.Cooling / 10);
    RefereeInformation.Realtime.Ammo1Heat -= (RefereeInformation.Ammo1Limit.Cooling / 10);
    RefereeInformation.Realtime.Ammo1Heat -= (RefereeInformation.Ammo2Limit.Cooling / 10);
}

void Ammo0HeatUpdateInterpolation(void)
{
    RefereeInformation.Realtime.Ammo0Heat += 10;
}

void Ammo1HeatUpdateInterpolation(void)
{
    RefereeInformation.Realtime.Ammo1Heat += 10;
}

void Ammo2HeatUpdateInterpolation(void)
{
    RefereeInformation.Realtime.Ammo2Heat += 100;
}


