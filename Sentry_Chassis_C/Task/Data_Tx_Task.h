#ifndef _Data_Tx_Task_H
#define _Data_Tx_Task_H
#include "main.h"
#include "Chassis_Task.h"
#include "bsp_can.h"

typedef struct
{

	uint8_t mode_data;
	fp32 set_yaw_angle;
	fp32 set_pitch_angle;
	
}Remote_Control_Data_Typedef;


typedef enum 
{
    CAMP_ERROR      =   0x00,
    RED             =   0x01,
    BLUE            =   0x02,
} camp_e;

typedef struct
{
    uint16_t    max_heat;
    uint16_t    cooling_heat;
    uint16_t    current_heat;
    uint16_t    max_rate;
    fp32        current_rate;
    camp_e      camp;
}shoot_referee_t;

typedef struct
{
	
	shoot_referee_t shoot_referee_data;
	
}CAN_Pack_Typedef;

void Get_Remote_Data(const RC_ctrl_t *rc_data);
void Get_Chassis_Data(uint8_t Chassis_Mode);
void Get_Robot_Data(uint8_t *rx_data);






#endif
