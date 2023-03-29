#include "Chassis_Task.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "rand_num.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "main.h"
#include "bsp_timer.h"
#include "ChassisPowerBehaviour.h"





#define BRAKE_ENABLE
#define POWER_LOOP_ENABLE

Chassis_Typedef Chassis;
fp32 Power_Pid[3],Brake_Pid[3];
uint8_t Power_Loop_Stop_Flag;
extern uint32_t systerm_time;
void Chassis_Task_Start(void const * argument)
{
	Chassis_init(&Chassis);
  for(;;)
  {
		Chassis.Chassis_RC = get_remote_control_point();
		Rand_Move(&Chassis);
		Detect(&Chassis);
	  Power_Calc(&Chassis);
    osDelay(1);
  }
}

/**
 *函数入口参数：(Chassis_Typedef *)底盘控制结构体
 *函数出口参数：无
 *变量说明：
 *函数说明：参数初始化
**/
void Chassis_init(Chassis_Typedef *chassis)
{
	chassis->Chassis_Mode_Flag.Current_Control_Mode = CHASSIS_WEEK;
  chassis->Power_Motor_Limit_Speed = POWER_MOTOR_HIGH_SPEED;

	chassis->Chassis_FeedBack_Data.Power_Motor = get_can1_motor_measure_point(0);
	chassis->Chassis_FeedBack_Data.Brake_Motor = get_can1_motor_measure_point(1);
	
	chassis->Chassis_RC = get_remote_control_point();
	
	chassis->Chassis_Control_Data.Power_Motor_SetSpeed = 0;
	chassis->Chassis_Control_Data.Brake_Motor_SetSpeed = 0;
	chassis->Chassis_Mode_Flag.Current_Set_Direction   = 1;
	chassis->Chassis_Control_Data.Power_Motor_Current  = 0;
	chassis->Chassis_Control_Data.Brake_Motor_Current  = 0;
	
	Power_Pid[0] = KP_POWER_STOP;
	Power_Pid[1] = KI_POWER_STOP;
	Power_Pid[2] = KD_POWER_STOP;
	
	Brake_Pid[0] = KP_BRAKE;
	Brake_Pid[1] = KI_BRAKE;
	Brake_Pid[2] = KD_BRAKE;
	
	PID_init(&(chassis->Chassis_Pid.Power_Motor_Pid),PID_POSITION,Power_Pid,MAX_POWER_OUT,MAX_POWER_IOUT);
	PID_init(&(chassis->Chassis_Pid.Brake_Motor_Pid),PID_POSITION,Power_Pid,MAX_BRAKE_OUT,MAX_BRAKE_IOUT);
	
	Power_Motor_Pid_init(chassis);
}
/**
 *函数入口参数：(Chassis_Typedef *)底盘控制结构体
 *函数出口参数：无
 *变量说明：
 *函数说明：pid参数初始化
**/
void Power_Motor_Pid_init(Chassis_Typedef *chassis)
{
	switch(chassis->Chassis_Mode_Flag.Current_Control_Mode)
	{
		case CHASSIS_WEEK:
		{
			Power_Pid[0] = 0.0f;
			Power_Pid[1] = 0.0f;
			Power_Pid[2] = 0.0f;
			break;
		}
		case CHASSIS_STOP:
		{
			Power_Pid[0] = KP_POWER_STOP;
			Power_Pid[1] = KI_POWER_STOP;
			Power_Pid[2] = KD_POWER_STOP;
			break;
		}
		case CHASSIS_SLOW:
		{
			Power_Pid[0] = KP_POWER_SLOW;
			Power_Pid[1] = KI_POWER_SLOW;
			Power_Pid[2] = KD_POWER_SLOW;
			break;
		}
		case CHASSIS_FAST:
		{
			Power_Pid[0] = KP_POWER_FAST;
			Power_Pid[1] = KI_POWER_FAST;
			Power_Pid[2] = KD_POWER_FAST;
			break;
		}
	}
	PID_init(&(chassis->Chassis_Pid.Power_Motor_Pid),PID_POSITION,Power_Pid,MAX_POWER_OUT,MAX_POWER_IOUT);
}

/**
 *函数入口参数：(Chassis_Typedef *)底盘控制结构体
 *函数出口参数：无
 *变量说明：
 *函数说明：速度值设定
**/
void Detect(Chassis_Typedef *chassis)
{  
	Direction_Detect(chassis);
	Brake_Detect(chassis);
	if(chassis->Chassis_Mode_Flag.Current_Control_Mode != CHASSIS_BRAKE)
	{
		chassis->Chassis_Mode_Flag.Current_Control_Mode = Mode_Updata(chassis);
	}
	
	switch(chassis->Chassis_Mode_Flag.Current_Control_Mode)
	{
		case CHASSIS_WEEK:
		{
			chassis->Chassis_Control_Data.Power_Motor_SetSpeed = 0;
			chassis->Chassis_Control_Data.Brake_Motor_SetSpeed = 0;
			break;
		}
		case CHASSIS_STOP:
		{
			chassis->Chassis_Control_Data.Power_Motor_SetSpeed = 0;
			chassis->Chassis_Control_Data.Brake_Motor_SetSpeed = 0;
			break;
		}
		case CHASSIS_SLOW:
		{
			chassis->Chassis_Control_Data.Power_Motor_SetSpeed = chassis->Chassis_Mode_Flag.Current_Set_Direction*Chassis.Power_Motor_Limit_Speed;
			chassis->Chassis_Control_Data.Brake_Motor_SetSpeed = 0;
			break;
		}
		case CHASSIS_FAST:
		{
			chassis->Chassis_Control_Data.Power_Motor_SetSpeed = chassis->Chassis_Mode_Flag.Current_Set_Direction*Chassis.Power_Motor_Limit_Speed;
			chassis->Chassis_Control_Data.Brake_Motor_SetSpeed = 0;
			break;
		}
		case CHASSIS_BRAKE:
		{
			//刹车模式下，将动力轮速度置为零
			chassis->Chassis_Control_Data.Brake_Motor_SetSpeed = Brake_Mode(chassis);
			chassis->Chassis_Control_Data.Power_Motor_SetSpeed = 0;
			break;
		}
		case CHASSIS_AUTO:
		{
			chassis->Chassis_Control_Data.Power_Motor_SetSpeed = chassis->Chassis_Mode_Flag.Current_Set_Direction*Chassis.Power_Motor_Limit_Speed;
			break;
		}
		default :
		{
			chassis->Chassis_Control_Data.Power_Motor_SetSpeed = 0;
			chassis->Chassis_Control_Data.Brake_Motor_SetSpeed = 0;
		}
		
		break;
	}
	Mode_Switch(chassis);
	
}
/**
 *函数入口参数：(Chassis_Typedef *)底盘控制结构体
 *函数出口参数：无
 *变量说明：
 *函数说明：检测底盘运动电机速度方向
**/
void Direction_Detect(Chassis_Typedef *chassis)
{
	if(chassis->Chassis_FeedBack_Data.Power_Motor->speed_rpm > 5)
	{
		chassis->Chassis_Mode_Flag.Current_Move_Direction = LEFT;
	}
	else if(chassis->Chassis_FeedBack_Data.Power_Motor->speed_rpm < -5)
	{
		chassis->Chassis_Mode_Flag.Current_Move_Direction = RIGHT;
	}
	else chassis->Chassis_Mode_Flag.Current_Move_Direction = 0;
	
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9) == 0)
	{
		chassis->Chassis_Mode_Flag.Current_Set_Direction = LEFT;
	}
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11) == 0)
	{
		chassis->Chassis_Mode_Flag.Current_Set_Direction = RIGHT;
	}
}

/**
 *函数入口参数：(Chassis_Typedef *)底盘控制结构体
 *函数出口参数：无
 *变量说明：
 *函数说明：检测是否需要进行刹车
**/
uint16_t brake_time_counter = 0;
void Brake_Detect(Chassis_Typedef *chassis)
{
	if(brake_time_counter >0)
	{
		brake_time_counter--;
	}
	//啓動刹車
	if(chassis->Chassis_Mode_Flag.Current_Move_Direction != chassis->Chassis_Mode_Flag.Current_Set_Direction
     &&chassis->Chassis_Mode_Flag.Current_Move_Direction != 0
	   &&chassis->Chassis_Mode_Flag.Current_Control_Mode != CHASSIS_WEEK)
	{
		chassis->Chassis_Mode_Flag.Current_Control_Mode = CHASSIS_BRAKE;
	}
	//当速度同向时或速度为零的瞬间，停止刹车模式
	if(chassis->Chassis_Mode_Flag.Current_Control_Mode == CHASSIS_BRAKE
		&&(chassis->Chassis_Mode_Flag.Current_Move_Direction == 0
	  ||chassis->Chassis_Mode_Flag.Current_Move_Direction == chassis->Chassis_Mode_Flag.Current_Set_Direction))
	{
		brake_time_counter = 50;
		chassis->Chassis_Mode_Flag.Current_Control_Mode = Mode_Updata(chassis);
	}
}
/**
 *函数入口参数：(Chassis_Typedef *)底盘控制结构体
 *函数出口参数：无
 *变量说明：Chassis_Typedef 电机底盘结构体
 *函数说明：给出刹车电流
**/
int Brake_Mode(Chassis_Typedef *chassis)
{
	if(chassis->Chassis_Mode_Flag.Current_Control_Mode == CHASSIS_BRAKE)
	{
		if(chassis->Chassis_Mode_Flag.Current_Set_Direction == LEFT)
		{
			return LEFT*BRAKE_MOTOR_SPEED;
		}
		else	if(chassis->Chassis_Mode_Flag.Current_Set_Direction == RIGHT)
		{
			return RIGHT*BRAKE_MOTOR_SPEED;
		}
	}
	return 0;
}

/**
 *函数入口参数：(Chassis_Typedef *)底盘控制结构体
 *函数出口参数：无
 *变量说明:
 *函数说明: 左上右上    高速运行
						左上右中   低速调试
						左上右下		 无力

						左下右下				全自动

**/
uint8_t Mode_Updata(Chassis_Typedef *chassis)
{
	switch(chassis->Chassis_RC->rc.s[LEFT_CHANNEL])
	{
		case RC_SW_MID:
		{
			switch(chassis->Chassis_RC->rc.s[RIGHT_CHANNEL])
			{
				case RC_SW_UP:
				{
					return CHASSIS_FAST;
				}
				case RC_SW_MID:
				{
					return CHASSIS_SLOW;
				}
				case RC_SW_DOWN:
				{
					return CHASSIS_WEEK;
				}
				default :return CHASSIS_WEEK;
			}
			break;
		}
		default : return CHASSIS_WEEK;
		break;
	}
}


/**
 *函数入口参数：(Chassis_Typedef *)底盘控制结构体
 *函数出口参数：无
 *变量说明:
 *函数说明:此函数用于模式切换
**/
void Mode_Switch(Chassis_Typedef *chassis)
{
	//检测当前是否存在控制转换
	if(chassis->Chassis_Mode_Flag.Last_Control_Mode != chassis->Chassis_Mode_Flag.Current_Control_Mode)
	{
		if(chassis->Chassis_Mode_Flag.Last_Control_Mode != CHASSIS_BRAKE)
		{
			chassis->Chassis_Mode_Flag.Translation_Flag = TRANSLATING_FLAG;
			chassis->Chassis_Mode_Flag.Last_Control_Mode = chassis->Chassis_Mode_Flag.Current_Control_Mode;
		}
	}
	//当前存在控制变换,速度置0
	if(chassis->Chassis_Mode_Flag.Translation_Flag == TRANSLATING_FLAG)
	{
		chassis->Chassis_Control_Data.Power_Motor_SetSpeed = 0;
		chassis->Chassis_Control_Data.Brake_Motor_SetSpeed = 0;		//状态转换结束
		chassis->Chassis_Mode_Flag.Translation_Flag = TRANSLAT_STOP_FLAG;
	}
	Power_Motor_Pid_init(chassis);
}
/**
 *函数入口参数：(Chassis_Typedef *)底盘控制结构体
 *函数出口参数：无
 *变量说明:
 *函数说明:用于电机控制电流计算
**/
void Power_Calc(Chassis_Typedef *chassis)
{
	if(chassis->Chassis_Mode_Flag.Current_Control_Mode != CHASSIS_WEEK)
	{
		if(chassis->Chassis_Mode_Flag.Current_Control_Mode != CHASSIS_BRAKE)
		{

#ifdef POWER_LOOP_ENABLE			
			ChassisPowerControl(chassis);
#else
		  chassis->Chassis_Control_Data.Power_Motor_Current = PID_calc(&(chassis->Chassis_Pid.Power_Motor_Pid),\
																												chassis->Chassis_FeedBack_Data.Power_Motor->speed_rpm,chassis->Chassis_Control_Data.Power_Motor_SetSpeed);
#endif
#ifdef BRAKE_ENABLE
		  //刹車歸位
			if(brake_time_counter>0)
			{
				chassis->Chassis_Control_Data.Brake_Motor_Current = -chassis->Chassis_Mode_Flag.Current_Move_Direction*130 ;
			}
			else chassis->Chassis_Control_Data.Brake_Motor_Current = PID_calc(&(chassis->Chassis_Pid.Brake_Motor_Pid),\
																												       chassis->Chassis_FeedBack_Data.Brake_Motor->speed_rpm,chassis->Chassis_Control_Data.Brake_Motor_SetSpeed);;
		}
		else if(chassis->Chassis_Mode_Flag.Current_Control_Mode == CHASSIS_BRAKE)
		{
//			chassis->Chassis_Control_Data.Power_Motor_Current = -chassis->Chassis_Mode_Flag.Current_Move_Direction*500;
			chassis->Chassis_Control_Data.Brake_Motor_Current = PID_calc(&(chassis->Chassis_Pid.Brake_Motor_Pid),\
																												  chassis->Chassis_FeedBack_Data.Brake_Motor->speed_rpm,chassis->Chassis_Control_Data.Brake_Motor_SetSpeed);
#endif
			//计算BRAKE_MOTOR的电流
		}
#ifndef BRAKE_ENABLE
		//没有刹车
#ifdef POWER_LOOP_ENABLE
		ChassisPowerControl(chassis);
#else
		chassis->Chassis_Control_Data.Power_Motor_Current = PID_calc(&(chassis->Chassis_Pid.Power_Motor_Pid),\
																											  chassis->Chassis_FeedBack_Data.Power_Motor->speed_rpm,chassis->Chassis_Control_Data.Power_Motor_SetSpeed);
#endif
#endif
	}
	if(chassis->Chassis_Mode_Flag.Current_Control_Mode == CHASSIS_WEEK)
	{
		//将POWER_MOTOR的电流置零
		chassis->Chassis_Control_Data.Power_Motor_Current = 0;
		//将Brake_MOTOR的电流置零
		chassis->Chassis_Control_Data.Brake_Motor_Current = 0;
	}
}


void Rand_Move(Chassis_Typedef *chassis)
{
	//更新当前随机数种子
	
	chassis->Chassis_Rand_Move.rand_seed = systerm_time;

	if(rand_num_flag == 0 
		&& chassis->Chassis_Rand_Move.rand_mode_enable_flag
		&& HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9) 
	  && HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11))
	{
		rand_num_flag = 100;
		srand(chassis->Chassis_Rand_Move.rand_seed);
		chassis->Chassis_Rand_Move.current_num = rand()%10;
		if(chassis->Chassis_Rand_Move.current_num > 6)
		{
				chassis->Chassis_Mode_Flag.Current_Set_Direction = -chassis->Chassis_Mode_Flag.Current_Set_Direction;
		}
	}
	
}
void Auto_Mode(Chassis_Typedef *chassis)
{
	if(chassis->Chassis_Mode_Flag.Current_Control_Mode == CHASSIS_AUTO)
	{
		Rand_Move(chassis);
	}
}








