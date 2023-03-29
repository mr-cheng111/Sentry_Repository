#ifndef CAN_PACKET_H
#define CAN_PACKET_H

#include "struct_typedef.h"
#include "main.h"

#define IMU_PACKET_TIME_ID                          0x100
#define IMU_PACKET_DATA0_ID                         0x102
#define IMU_PACKET_DATA1_ID                         0x103

#define AIMBOT_STATE_NODE_ID                        0x106
#define AIMBOT_DATA_NODE_ID                         0x108

#define GINBAL_REQUEST_STATE_ID                     0x110

#define CAN_RC_CONTROL_ID 							0x091
#define CHASSIS_MODE_ID 							0x092
#define REFEREE_DATA_ID 							0x093


#define REFEREE_POWER_HEAT_NODE_0_ID                0x120
#define REFEREE_POWER_HEAT_NODE_1_ID                0x121
#define REFEREE_AMMO_SPEED_NODE_0_ID                0x122
#define REFEREE_AMMO_SPEED_NODE_1_ID                0x123
#define REFEREE_AMMO_SPEED_NODE_2_ID                0x124
#define REFEREE_AMMO_LIMIT_NODE_0_ID                0x125
#define REFEREE_AMMO_LIMIT_NODE_1_ID                0x126
#define REFEREE_AMMO_LIMIT_NODE_2_ID                0x127
#define REFEREE_SELF_STATE_NODE                     0x129

typedef __PACKED_STRUCT
{
    uint32_t    TimeStamp;
    fp32        Quaternion[4];
} ImuPacketNormal_t;

typedef __PACKED_STRUCT
{
    uint8_t     AimbotState;
    uint8_t     AimbotTarget;
} AimbotStateNoraml_t;

typedef __PACKED_STRUCT
{
    int16_t     PitchRelativeAngle;
    int16_t     YawRelativeAngle;
    uint32_t    SystemTimer;
} AimbotCommandNoraml_t;

typedef __PACKED_STRUCT
{
    uint8_t     AimbotRequest;
    int16_t     ChassisMoveXRequest;
    int16_t     ChassisMoveYRequest;
    uint8_t     ChassisStateRequest;
    uint8_t     GimbalState;
    uint8_t     Reserve;
} GimbalRequestState_t;

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

typedef __PACKED_STRUCT{
    uint8_t     RobotID;
    uint8_t     PowerState;
    uint16_t    Blood;
    uint16_t    BloodLimit;
} RefereeSelfState_t;

typedef __PACKED_STRUCT
{
    uint8_t     GameState;
    uint8_t     PowerState;
    uint16_t    Blood;
    uint16_t    BloodLimit;
	
} RefereeGameState_t;

typedef __PACKED_STRUCT
{	
	uint8_t rx_data0;
    uint8_t rx_data1;
    uint8_t rx_data2;
	uint8_t rx_data3;
	uint8_t rx_data4;
	uint8_t rx_data5;
	uint8_t rx_data6;
	uint8_t rx_data7;

} CAN_RC_DATA_t;
typedef __PACKED_STRUCT
{
   int16_t ch[5];
   char s[2];
				
} CAN_RC_ctrl_t;

//比赛状态标志位
//bit0 比赛开始状态        0未开始 1开始
//bit1 是否开启保卫模式    0未开始 1开始
//bit2 点射或扫射				   0未开始 1开始
typedef __PACKED_STRUCT
{
	uint8_t Game_Start_State;
	uint8_t Defend_Mode;
	uint8_t Point_Fire_Mode;
	uint8_t buf3;
	uint8_t buf4;
	uint8_t buf5;
	uint8_t buf6;
	uint8_t buf7;
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
typedef enum
{
	Chassis_Week = 0,
	Chassis_Stop,
	Chassis_Fast,
	Chassis_Brake

}Chassis_Mode_Type;

extern uint8_t Chassis_Mode;
typedef __PACKED_STRUCT
{
	uint8_t bullet_remaining_num_17mm;
	uint8_t chassis_power_buffer;
    uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id2_17mm_cooling_heat;
	uint8_t Armor_Type : 4;
}Robot_Data_t;



extern Game_State_t Game_State;





#endif
