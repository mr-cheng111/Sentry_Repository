#include "ChassisPowerBehaviour.h"
#include "Chassis_Task.h"
#include "Referee_Behaviour.h"
#include "user_lib.h"
#include "math.h"
#include "IT_Sever.h"
extern frame_header_struct_t referee_receive_header;
extern frame_header_struct_t referee_send_header;

extern ext_game_state_t game_state;
extern ext_game_result_t game_result;
extern ext_game_robot_HP_t game_robot_HP_t;

extern ext_game_robot_status_t robot_state;
extern ext_power_heat_data_t power_heat_data_t;
extern aerial_robot_energy_t robot_energy_t;
extern ext_robot_hurt_t robot_hurt_t;
extern ext_shoot_data_t shoot_data_t;
extern ext_bullet_remaining_t bullet_remaining_t;

fp32 chassis_total_current = 0;
fp32 chassis_buffer_current = 0;

float BufferEnergy = 200; //底盘缓冲能量
extern uint8_t Power_Loop_Stop_Flag;;

#define Least_Power_Scale 0.15

//裁判系统得到数值
float power_limit = 0;
fp32 chassis_current_power = 0.0f;
fp32 chassis_power_buffer = 0.0f;
int i,times = 0;
//一阶低通滤波器
first_order_filter_type_t filter;
fp32 num[1] = {0};
//电流限制
float MomentCurrent = 0.0f,MomentPower0 = 0.0f;
float k = 9.9999999992*1e-05;
float last_speed = 0;
float power_scale,referee_powermax;
float MomentPower = 0;
void ChassisPowerInit();
void CapCharge(float Percentage);
void ChassisReduceRate();
fp32 chassis_powerloop(Chassis_Typedef *Chassis);
void ChassisPowerControl(Chassis_Typedef *Chassis);
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1]);
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
//更新数据
void ChassisPowerInit()
{
	referee_powermax = robot_state.chassis_power_limit;
	chassis_power_buffer = Robot_Data.chassis_power_buffer;
}

void ChassisPowerControl(Chassis_Typedef *chassis)
{
	ChassisPowerInit();
	chassis_powerloop(chassis);
}
fp32 chassis_powerloop(Chassis_Typedef *chassis)
{
	//扭矩电流计算
	MomentCurrent = fabs((float)chassis->Chassis_Control_Data.Power_Motor_Current);
	//功率计算
	MomentPower = fabs(fabs((float)chassis->Chassis_FeedBack_Data.Power_Motor->speed_rpm) - fabs((float)chassis->Chassis_Control_Data.Power_Motor_SetSpeed))\
																												*2*3.1415926535897932384626433832795/60*k;
	//当扭矩功率大于0且小于一定值，则更新扭矩功率为此刻裁判系统功率
	if( MomentPower > chassis_current_power && MomentPower - chassis_current_power < 2)
	{
		  MomentPower = chassis_current_power;
	}
	chassis->Chassis_Control_Data.Power_Motor_Current = PID_calc(&(chassis->Chassis_Pid.Power_Motor_Pid),\
	chassis->Chassis_FeedBack_Data.Power_Motor->speed_rpm,chassis->Chassis_Control_Data.Power_Motor_SetSpeed);
	//当此时底盘缓冲能量小于50时,此时应该认为是极限状态，应给出一个非常大的功率回复方案，并且严格限制瞬间反向电流,并且停止随机运动，使其回复功率
	if(chassis_power_buffer < 80)
	{
	  chassis->Chassis_Rand_Move.rand_mode_enable_flag = 0;
	  Chassis.Power_Motor_Limit_Speed = POWER_MOTOR_SLOW_SPEED;
	  power_scale = Least_Power_Scale;
		
	  chassis->Chassis_Control_Data.Power_Motor_Current = power_scale*PID_calc(&(chassis->Chassis_Pid.Power_Motor_Pid),\
	  chassis->Chassis_FeedBack_Data.Power_Motor->speed_rpm,chassis->Chassis_Control_Data.Power_Motor_SetSpeed);
		return 0;
	}
	if(chassis_power_buffer < 100)
	{
	  chassis->Chassis_Rand_Move.rand_mode_enable_flag = 0;
	  Chassis.Power_Motor_Limit_Speed = POWER_MOTOR_SLOW_SPEED;
	  power_scale = Least_Power_Scale;
		
		if(abs_int(chassis->Chassis_FeedBack_Data.Power_Motor->speed_rpm) > abs_int(chassis->Chassis_Control_Data.Power_Motor_SetSpeed))
		{
			chassis->Chassis_Control_Data.Power_Motor_Current = power_scale*PID_calc(&(chassis->Chassis_Pid.Power_Motor_Pid),\
			chassis->Chassis_FeedBack_Data.Power_Motor->speed_rpm,chassis->Chassis_Control_Data.Power_Motor_SetSpeed);
		}
		else chassis->Chassis_Control_Data.Power_Motor_Current = 0;
		return 0;
	}
	if(chassis_power_buffer < 170)
	{
		Chassis.Power_Motor_Limit_Speed = POWER_MOTOR_SLOW_SPEED;
		chassis->Chassis_Rand_Move.rand_mode_enable_flag = 0;
		power_scale = chassis_power_buffer/170;
		Chassis.Power_Motor_Limit_Speed = power_scale*POWER_MOTOR_HIGH_SPEED;
		if(abs_int(chassis->Chassis_FeedBack_Data.Power_Motor->speed_rpm) > abs_int(chassis->Chassis_Control_Data.Power_Motor_SetSpeed))
		{
			chassis->Chassis_Control_Data.Power_Motor_Current = power_scale*PID_calc(&(chassis->Chassis_Pid.Power_Motor_Pid),\
			chassis->Chassis_FeedBack_Data.Power_Motor->speed_rpm,chassis->Chassis_Control_Data.Power_Motor_SetSpeed);
		}
		else chassis->Chassis_Control_Data.Power_Motor_Current = 0;
		return 0;
	}
		//当此时底盘缓冲能量小于180时,此时应该认为是较低能状态，给出功率回复方案,大约3W左右
	else if(chassis_power_buffer < 180)
	{
		chassis->Chassis_Rand_Move.rand_mode_enable_flag = 1;
		power_scale = chassis_power_buffer/180;
		for(uint8_t counter = 0;(counter < 20) && ((MomentPower - referee_powermax < -8) || (MomentPower - referee_powermax < -6)) ; counter ++)
		{
			power_scale = referee_powermax/MomentPower*chassis_power_buffer/200;
			if(power_scale > 1)
			{
				power_scale = 1;
			}
			chassis->Chassis_Control_Data.Power_Motor_SetSpeed *= power_scale;
				
			chassis->Chassis_Control_Data.Power_Motor_Current = PID_calc(&(chassis->Chassis_Pid.Power_Motor_Pid),\
			chassis->Chassis_FeedBack_Data.Power_Motor->speed_rpm,chassis->Chassis_Control_Data.Power_Motor_SetSpeed);
		  //虚拟功率计算,0.9是预测达到的速度值，
		  MomentPower = fabs(fabs((float)chassis->Chassis_FeedBack_Data.Power_Motor->speed_rpm) - fabs((float)chassis->Chassis_Control_Data.Power_Motor_SetSpeed))\
								    *2*3.1415926535897932384626433832795/60*k;
		}
		return 0;
	}
	else 
	{
			power_scale = chassis_power_buffer/200;
			Chassis.Power_Motor_Limit_Speed = POWER_MOTOR_HIGH_SPEED;
		    chassis->Chassis_Rand_Move.rand_mode_enable_flag = 1;
			Chassis.Power_Motor_Limit_Speed = POWER_MOTOR_HIGH_SPEED;
			chassis->Chassis_Control_Data.Power_Motor_Current = PID_calc(&(chassis->Chassis_Pid.Power_Motor_Pid),\
			chassis->Chassis_FeedBack_Data.Power_Motor->speed_rpm,chassis->Chassis_Control_Data.Power_Motor_SetSpeed);
		return 0;
	}
}
