#ifndef IT_SEVER
#define IT_SEVER

#include "main.h"
#include "struct_typedef.h"
#include "bsp_Referee.h"
#include "Referee_Behaviour.h"

#define CHASSIS_MODE_ID 0x092
#define RC_DATA_ID 		0x091
#define ROBOT_DATA_ID   0x129
#define REFEREE_DATA_ID 0x122


typedef enum
{
	Chassis_Week = 1,
	Chassis_Stop,
	Chassis_Fast,
	Chassis_Brake

}Sentry_Chassis_Mode_Type;

typedef __PACKED_STRUCT{
    uint8_t     RobotID;
    uint8_t     PowerState;
    uint16_t    Blood;
    uint16_t    BloodLimit;
} RefereeSelfState_t;

typedef __PACKED_STRUCT
{
    uint16_t    ChassisVoltage;
    uint16_t    ChassisCurrent;
    fp32        ChassisPower;
    uint16_t    ChassisBufferEnergy;
    uint16_t    Ammo0Heat;
    uint16_t    Ammo1Heat;
    uint16_t    Ammo2Heat;
} RefereeChassisPowerShootHeat_t;

typedef __PACKED_STRUCT
{
    uint16_t    Cooling;
    uint16_t    Heat;
    uint16_t    Speed;
} RefereeAmmoLimit_t;

typedef struct
{
    RefereeChassisPowerShootHeat_t      Realtime;
    fp32                                Ammo0Speed;
    fp32                                Ammo1Speed;
    fp32                                Ammo2Speed;
    RefereeAmmoLimit_t                  Ammo0Limit;
    RefereeAmmoLimit_t                  Ammo1Limit;
    RefereeAmmoLimit_t                  Ammo2Limit;
    RefereeSelfState_t                  RobotState;
} RefereeInformation_t;



//比赛状态标志位
//bit0 比赛开始状态        	0未开始 1开始
//bit1 是否开启保卫模式    	0未开始 1开始
//bit2 点射或扫射			0未开始 1开始
typedef __PACKED_STRUCT
{
	uint8_t Game_Start_State;
	uint8_t Defend_Mode;
	uint8_t Point_Fire_Mode;
    uint8_t Chassis_Mode;
}Game_State_t;

typedef enum
{
	Game_START = 1,
	Game_STOP,
	Sentry_Defend,
	Sentry_Escape,
	Sentry_Point_Fire,
	Sentry_Brust_Fire,

}Game_STATE_TYPE;




typedef __PACKED_STRUCT
{
	uint8_t bullet_remaining_num_17mm;
	uint8_t chassis_power_buffer;
    uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id2_17mm_cooling_heat;
	uint8_t Armor_Type : 4;
}Robot_Data_t;






extern Game_State_t Game_State;
extern Robot_Data_t Robot_Data;
extern RefereeInformation_t RefereeInformation;



void Get_Referee_Data(uint8_t *rx_data);
void Get_Robot_Data(uint8_t *rx_data);







#endif