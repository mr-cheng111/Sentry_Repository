#ifndef _CHASSIS_TASK_H
#define _CHASSIS_TASK_H
#include "struct_typedef.h"
#include "bsp_motor.h"
#include "bsp_rc.h"
#include "pid.h"




//控制状态

#define	CHASSIS_WEEK 	0x0
#define	CHASSIS_STOP  0x1
#define	CHASSIS_SLOW	0x2
#define	CHASSIS_FAST	0x3
#define	CHASSIS_BRAKE	0x4
#define	CHASSIS_AUTO	0x5

//标志位
#define LEFT   1
#define RIGHT -1

#define TRANSLATING_FLAG 1
#define TRANSLAT_STOP_FLAG 0

#define MAX_POWER_OUT  16384.0f
#define MAX_POWER_IOUT  2000.0f 

#define KP_POWER_STOP 20.0f
#define KI_POWER_STOP 0.0f
#define KD_POWER_STOP 0.0f

#define KP_POWER_SLOW 10.0f
#define KI_POWER_SLOW 0.0f
#define KD_POWER_SLOW 0.0f

#define KP_POWER_FAST 10.0f
#define KI_POWER_FAST 0.0f
#define KD_POWER_FAST 0.0f

#define MAX_BRAKE_OUT  10000.0f
#define MAX_BRAKE_IOUT  2000.0f 

#define KP_BRAKE 0.0f
#define KI_BRAKE 0.0f
#define KD_BRAKE 0.0f

#define LEFT_CHANNEL  1
#define RIGHT_CHANNEL 0
 
#define POWER_MOTOR_SLOW_SPEED 4000
#define POWER_MOTOR_HIGH_SPEED 7000
#define POWER_MOTOR_AUTO_SPEED 5000




#define BRAKE_MOTOR_SPEED 6000

//控制状态

typedef struct _CHASSIS_FEEDBACK_DATA
{
	const motor_measure_t *Power_Motor;
	const motor_measure_t *Brake_Motor;

}__IO Chassis_FeedBack_Typedef;

typedef struct _CHASSIS_CONTROL_DATA
{
	int16_t Power_Motor_SetSpeed;
	int16_t Brake_Motor_SetSpeed;
	
	int16_t Power_Motor_Current;
	int16_t Brake_Motor_Current;
}Chassis_Control_Typedef;

typedef struct _CHASSIS_PID_DATA
{
	pid_type_def Power_Motor_Pid;
	pid_type_def Brake_Motor_Pid;
	
}Chassis_Pid_Typedef;

typedef struct _CHASSIS_MODE_DATA
{
	uint8_t Current_Control_Mode;
	uint8_t Translation_Flag;
	int Current_Set_Direction;
	//当前电机的运动方向
	int Current_Move_Direction;
	int Direction_Triggle_Flag;
	
	int Last_Control_Mode;
}Chassis_Mode_Typedef;

typedef struct _CHASSIS_RAND_MOVE
{
	int current_num;
	uint32_t rand_seed;
	uint8_t rand_mode_enable_flag;
}Chassis_Rand_Typedef;

typedef struct _CHASSIS_DATA
{
	Chassis_Mode_Typedef Chassis_Mode_Flag;
	
	__IO Chassis_FeedBack_Typedef Chassis_FeedBack_Data;
	Chassis_Control_Typedef Chassis_Control_Data;
	Chassis_Pid_Typedef Chassis_Pid;
	RC_ctrl_t *Chassis_RC;
	
	Chassis_Rand_Typedef Chassis_Rand_Move;
	
	int Power_Motor_Limit_Speed;
}Chassis_Typedef;



extern Chassis_Typedef Chassis;


void Chassis_init(Chassis_Typedef *chassis);
void Power_Motor_Pid_init(Chassis_Typedef *chassis);
void Detect(Chassis_Typedef *chassis);
void Mode_Switch(Chassis_Typedef *chassis);
void Power_Calc(Chassis_Typedef *chassis);
uint8_t Mode_Updata(Chassis_Typedef *chassis);
void Direction_Detect(Chassis_Typedef *chassis);
void Brake_Detect(Chassis_Typedef *chassis);
int  Brake_Mode(Chassis_Typedef *chassis);
void Rand_Move(Chassis_Typedef *chassis);



#endif