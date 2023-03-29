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
#include "Referee_Behaviour.h"
#include "IT_Sever.h"

extern RefereeInformation_t RefereeInformation;
extern Game_State_t Game_State;
extern Robot_Data_t Robot_Data;

#define LEFT_GPIO_Pin GPIO_PIN_13
#define RIGHT_GPIO_Pin GPIO_PIN_11

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
 *������ڲ�����(Chassis_Typedef *)���̿��ƽṹ��
 *�������ڲ�������
 *����˵����
 *����˵����������ʼ��
**/
void Chassis_init(Chassis_Typedef *chassis)
{
	chassis->Chassis_Mode_Flag.Current_Control_Mode = CHASSIS_WEEK;
    chassis->Power_Motor_Limit_Speed = POWER_MOTOR_HIGH_SPEED;

	chassis->Chassis_FeedBack_Data.Power_Motor = get_can1_motor_measure_point(0);
	chassis->Chassis_FeedBack_Data.Brake_Motor = get_can1_motor_measure_point(2);
	
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
 *������ڲ�����(Chassis_Typedef *)���̿��ƽṹ��
 *�������ڲ�������
 *����˵����
 *����˵����pid������ʼ��
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
 *������ڲ�����(Chassis_Typedef *)���̿��ƽṹ��
 *�������ڲ�������
 *����˵����
 *����˵�����ٶ�ֵ�趨
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
		case CHASSIS_FAST:
		{
			chassis->Chassis_Control_Data.Power_Motor_SetSpeed = chassis->Chassis_Mode_Flag.Current_Set_Direction*Chassis.Power_Motor_Limit_Speed;
			chassis->Chassis_Control_Data.Brake_Motor_SetSpeed = 0;
			break;
		}
		case CHASSIS_BRAKE:
		{
			//ɲ��ģʽ�£����������ٶ���Ϊ��
			chassis->Chassis_Control_Data.Brake_Motor_SetSpeed = Brake_Mode(chassis);
			chassis->Chassis_Control_Data.Power_Motor_SetSpeed = 0;
			break;
		}
		default :
		{
			chassis->Chassis_Control_Data.Power_Motor_SetSpeed = 0;
			chassis->Chassis_Control_Data.Brake_Motor_SetSpeed = 0;
		}
		break;

	}
	
	
//	chassis->Chassis_Mode_Flag.Current_Control_Mode 
	Mode_Switch(chassis);
	
}
/**
 *������ڲ�����(Chassis_Typedef *)���̿��ƽṹ��
 *�������ڲ�������
 *����˵����
 *����˵�����������˶�����ٶȷ���
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
	
	if(HAL_GPIO_ReadPin(GPIOE,LEFT_GPIO_Pin) == 0)
	{
		chassis->Chassis_Mode_Flag.Current_Set_Direction = LEFT;
		rand_num_flag = 150;
	}
	if(HAL_GPIO_ReadPin(GPIOE,RIGHT_GPIO_Pin) == 0)
	{
		chassis->Chassis_Mode_Flag.Current_Set_Direction = RIGHT;
		rand_num_flag = 150;
	}
}

/**
 *������ڲ�����(Chassis_Typedef *)���̿��ƽṹ��
 *�������ڲ�������
 *����˵����
 *����˵��������Ƿ���Ҫ����ɲ��
**/
uint16_t brake_time_counter = 0;
void Brake_Detect(Chassis_Typedef *chassis)
{
	if(brake_time_counter >0)
	{
		brake_time_counter--;
	}
	//����ɲ܇
	if(chassis->Chassis_Mode_Flag.Current_Move_Direction != chassis->Chassis_Mode_Flag.Current_Set_Direction
	   &&chassis->Chassis_Mode_Flag.Current_Move_Direction != 0
	   &&chassis->Chassis_Mode_Flag.Current_Control_Mode != CHASSIS_WEEK)
	{
		chassis->Chassis_Mode_Flag.Current_Control_Mode = CHASSIS_BRAKE;
	}
	//���ٶ�ͬ��ʱ���ٶ�Ϊ���˲�䣬ֹͣɲ��ģʽ
	if(chassis->Chassis_Mode_Flag.Current_Control_Mode == CHASSIS_BRAKE
	   &&(chassis->Chassis_Mode_Flag.Current_Move_Direction == 0
	   ||chassis->Chassis_Mode_Flag.Current_Move_Direction == chassis->Chassis_Mode_Flag.Current_Set_Direction))
	{
		brake_time_counter = 50;
		chassis->Chassis_Mode_Flag.Current_Control_Mode = Mode_Updata(chassis);
	}
}
/**
 *������ڲ�����(Chassis_Typedef *)���̿��ƽṹ��
 *�������ڲ�������
 *����˵����Chassis_Typedef ������̽ṹ��
 *����˵��������ɲ������
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
 *������ڲ�����(Chassis_Typedef *)���̿��ƽṹ��
 *�������ڲ�������
 *����˵��:
 *����˵��: ��������    ��������
						��������   ���ٵ���
						��������   ����
						��������   ȫ�Զ�
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
				case RC_SW_MID:
				{
					return CHASSIS_FAST;
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
 *������ڲ�����(Chassis_Typedef *)���̿��ƽṹ��
 *�������ڲ�������
 *����˵��:
 *����˵��:�˺�������ģʽ�л�
**/
void Mode_Switch(Chassis_Typedef *chassis)
{
	//��⵱ǰ�Ƿ���ڿ���ת��
	if(chassis->Chassis_Mode_Flag.Last_Control_Mode != chassis->Chassis_Mode_Flag.Current_Control_Mode)
	{
		if(chassis->Chassis_Mode_Flag.Last_Control_Mode != CHASSIS_BRAKE)
		{
			chassis->Chassis_Mode_Flag.Translation_Flag = TRANSLATING_FLAG;
			chassis->Chassis_Mode_Flag.Last_Control_Mode = chassis->Chassis_Mode_Flag.Current_Control_Mode;
		}
	}
	//��ǰ���ڿ��Ʊ任,�ٶ���0
	if(chassis->Chassis_Mode_Flag.Translation_Flag == TRANSLATING_FLAG)
	{
		chassis->Chassis_Control_Data.Power_Motor_SetSpeed = 0;
		chassis->Chassis_Control_Data.Brake_Motor_SetSpeed = 0;		//״̬ת������
		chassis->Chassis_Mode_Flag.Translation_Flag = TRANSLAT_STOP_FLAG;
	}
	Power_Motor_Pid_init(chassis);
}
/**
 *������ڲ�����(Chassis_Typedef *)���̿��ƽṹ��
 *�������ڲ�������
 *����˵��:
 *����˵��:���ڵ�����Ƶ�������
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
		  //ɲ܇�wλ
		if(brake_time_counter>0)
		{
			chassis->Chassis_Control_Data.Brake_Motor_Current = -chassis->Chassis_Mode_Flag.Current_Move_Direction*500 ;
		}
		else chassis->Chassis_Control_Data.Brake_Motor_Current = PID_calc(&(chassis->Chassis_Pid.Brake_Motor_Pid),\
																 chassis->Chassis_FeedBack_Data.Brake_Motor->speed_rpm,chassis->Chassis_Control_Data.Brake_Motor_SetSpeed);
		}
		else if(chassis->Chassis_Mode_Flag.Current_Control_Mode == CHASSIS_BRAKE
//				&& HAL_GPIO_ReadPin(GPIOE,LEFT_GPIO_Pin)
//				&& HAL_GPIO_ReadPin(GPIOE,RIGHT_GPIO_Pin))
				)
		{

//			chassis->Chassis_Control_Data.Power_Motor_Current = -chassis->Chassis_Mode_Flag.Current_Move_Direction*500;

            chassis->Chassis_Control_Data.Power_Motor_Current = 0;

			chassis->Chassis_Control_Data.Brake_Motor_Current = PID_calc(&(chassis->Chassis_Pid.Brake_Motor_Pid),\
																chassis->Chassis_FeedBack_Data.Brake_Motor->speed_rpm,chassis->Chassis_Control_Data.Brake_Motor_SetSpeed);
#endif
			//����BRAKE_MOTOR�ĵ���
			}
		
#ifndef BRAKE_ENABLE
	//û��ɲ��
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
		//��POWER_MOTOR�ĵ�������
		chassis->Chassis_Control_Data.Power_Motor_Current = 0;
		//��Brake_MOTOR�ĵ�������
		chassis->Chassis_Control_Data.Brake_Motor_Current = 0;
	}
}


void Rand_Move(Chassis_Typedef *chassis)
{
//���µ�ǰ���������
	
	chassis->Chassis_Rand_Move.rand_seed = systerm_time;

	if(rand_num_flag == 0 
		&& chassis->Chassis_Rand_Move.rand_mode_enable_flag
		&& HAL_GPIO_ReadPin(GPIOE,LEFT_GPIO_Pin)
		&& HAL_GPIO_ReadPin(GPIOE,RIGHT_GPIO_Pin))
	{
		rand_num_flag = 200;
		srand(chassis->Chassis_Rand_Move.rand_seed);
		chassis->Chassis_Rand_Move.current_num = RNG_Get_RandomRange(1,100);
		if(chassis->Chassis_Rand_Move.current_num > 78)
		{
			chassis->Chassis_Mode_Flag.Current_Set_Direction = -chassis->Chassis_Mode_Flag.Current_Set_Direction;
		}
	}
	//��������֧�ַ�����װ�װ��ܵ��˺�����ʱ���з������
	if((Robot_Data.chassis_power_buffer >= 150)
	   && Robot_Data.Armor_Type != 0)
	{
		chassis->Chassis_Mode_Flag.Current_Set_Direction = -chassis->Chassis_Mode_Flag.Current_Set_Direction;
	}

}









