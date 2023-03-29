#include "Chassis.h"
#include "pid.h"
#include "bsp_can.h"
#include "bsp_rc.h"
#include "remote_control.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "variables.h"
#include "math.h"
#include "usart.h"
#include "math.h"



const motor_measure_t *yaw_motor_data;
chassis_feedback_typedef Chassis_FeedBack_Data;
chassis_typedef Chassis_Set_Data;
chassis_rc_control_typedef Chassis_Get_Control_Data;
chassis_speed_set Chassis_Speed;



void Chassis_Remote_Control_init(void);
void Chassis_PID_init(void);
void Chassis_init(chassis_feedback_typedef *init);
void Chassis_Current_calc(void);
void Chassis_Get_Control(void);
void Chassis_Get_Control_Speed(void);
void Get_FeedBack(void);

int sevor_error_angle = 0;

 fp32 cita;




void Chassis_Task_Start(void const * argument)
{
	Chassis_Remote_Control_init();
	Chassis_PID_init();
	Chassis_init(&Chassis_FeedBack_Data);
	yaw_motor_data = get_yaw_gimbal_motor_measure_point();
	
  for(;;)
  {
		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_9);
		shoot_flag = 1;
		Chassis_Get_Control();
		Get_FeedBack();
		Chassis_Get_Control_Speed();
		Chassis_Current_calc();
//		CAN_cmd_chassis(Chassis_Speed.W1,-Chassis_Speed.W2,-Chassis_Speed.W3,Chassis_Speed.W4);
		CAN_cmd_trigge(Chassis_Set_Data.give_trigge_current);
    osDelay(1);
  }

}
void Chassis_Remote_Control_init(void)
{
	Chassis_Get_Control_Data.chassis_ctrl = get_remote_control_point();
	Chassis_Get_Control_Data.Vx = 0;
	Chassis_Get_Control_Data.Vy = 0;
	Chassis_Get_Control_Data.Wz = 0;
	Chassis_Get_Control_Data.triggle = 0;
	Chassis_Speed.W1 = 0;
	Chassis_Speed.W2 = 0;
	Chassis_Speed.W3 = 0;
	Chassis_Speed.W4 = 0;
	Chassis_Speed.triggle = 0;
}
void Chassis_PID_init(void)
{
	//底盘四个电机的pid
	Chassis_Pid[0][0] = Chassis1_KP ;
	Chassis_Pid[1][0] = Chassis2_KP ;
	Chassis_Pid[2][0] = Chassis3_KP ;
	Chassis_Pid[3][0] = Chassis4_KP ;

	Chassis_Pid[0][1] = Chassis1_KI ;
	Chassis_Pid[1][1] = Chassis2_KI ;
	Chassis_Pid[2][1] = Chassis3_KI ;
	Chassis_Pid[3][1] = Chassis4_KI ;
	
	Chassis_Pid[0][2] = Chassis1_KD ;
	Chassis_Pid[1][2] = Chassis2_KD ;
	Chassis_Pid[2][2] = Chassis3_KD ;
	Chassis_Pid[3][2] = Chassis4_KD ;
	
	//合速度的pid三参数设定
	Vx_pid[0] = VX_P;
	Vx_pid[1] = VX_I;
	Vx_pid[2] = VX_D;
	
	Vy_pid[0] = VY_P;
	Vy_pid[1] = VY_I;
	Vy_pid[2] = VY_D;
	
	Wz_pid[0] = WZ_P;
	Wz_pid[1] = WZ_I;
	Wz_pid[2] = WZ_D;
	
	//拨盘pid
	trigge_angle_pid[0] = TRIGGE_ANGLE_KP;
	trigge_angle_pid[1] = TRIGGE_ANGLE_KI;
	trigge_angle_pid[2] = TRIGGE_ANGLE_KD;
	
	trigge_speed_pid[0] = TRIGGE_SPEED_KP;
	trigge_speed_pid[1] = TRIGGE_SPEED_KI;
	trigge_speed_pid[2] = TRIGGE_SPEED_KD;
	
	PID_init(&wheel1,PID_POSITION,Chassis_Pid[0],MAX_CHASSIS_OUT,MAX_CHASSIS_IOUT);
	PID_init(&wheel2,PID_POSITION,Chassis_Pid[1],MAX_CHASSIS_OUT,MAX_CHASSIS_IOUT);
	PID_init(&wheel3,PID_POSITION,Chassis_Pid[2],MAX_CHASSIS_OUT,MAX_CHASSIS_IOUT);
	PID_init(&wheel4,PID_POSITION,Chassis_Pid[3],MAX_CHASSIS_OUT,MAX_CHASSIS_IOUT);
	
	PID_init(&Vx_ref,PID_POSITION,Vx_pid,MAX_VX_OUT,MAX_VX_IOUT);
	PID_init(&Vy_ref,PID_POSITION,Vy_pid,MAX_VY_OUT,MAX_VY_IOUT);
	PID_init(&Wz_ref,PID_POSITION,Wz_pid,MAX_WZ_OUT,MAX_WZ_IOUT);
	
	PID_init(&trigger_angle,PID_POSITION,trigge_angle_pid,MAX_PID_TRIGGE_ANGLE_OUT,MAX_PID_TRIGGE_ANGLE_IOUT);
	PID_init(&trigger_speed,PID_POSITION,trigge_speed_pid,MAX_PID_TRIGGE_SPEED_OUT,MAX_PID_TRIGGE_SPEED_IOUT);
}

void Chassis_init(chassis_feedback_typedef *init)
{
	//获取电机数据指针
	init->chassis_motor_data1 = get_chassis_motor_measure_point(0);
	init->chassis_motor_data2 = get_chassis_motor_measure_point(1);
	init->chassis_motor_data3 = get_chassis_motor_measure_point(2);
	init->chassis_motor_data4 = get_chassis_motor_measure_point(3);
	init->chassis_motor_data5 = get_triggle_motor_measure_point();
}


//获取当前遥控器控制量
void Chassis_Get_Control(void)
{
	switch(Chassis_Get_Control_Data.chassis_ctrl->rc.s[1])
	{
		case ROTATING: 
		{
			sevor_error_angle = yaw_motor_data->ecd-4095+3200;
			if(sevor_error_angle > 4094)
			{
			 sevor_error_angle = -8195 + sevor_error_angle;
			}
			
			else if(sevor_error_angle < -4095)
			{
				sevor_error_angle = 8195 - sevor_error_angle;
			}
			cita = (fp32)sevor_error_angle/8195*2*PI;
			
			Chassis_Get_Control_Data.Vx =  VX_SPEED*Chassis_Get_Control_Data.chassis_ctrl->rc.ch[0]*cosf(cita)+VY_SPEED*Chassis_Get_Control_Data.chassis_ctrl->rc.ch[1]*sinf(cita);
			Chassis_Get_Control_Data.Vy = -VX_SPEED*Chassis_Get_Control_Data.chassis_ctrl->rc.ch[0]*sinf(cita)+VY_SPEED*Chassis_Get_Control_Data.chassis_ctrl->rc.ch[1]*cosf(cita);
			Chassis_Get_Control_Data.Wz = 20000;
			
			
			

			break;
		}
		
		case SERVO:
		{ 
			sevor_error_angle = yaw_motor_data->ecd-4095+3200;
			if(sevor_error_angle > 4094)
			{
			 sevor_error_angle = -8195 + sevor_error_angle;
			}
			
			else if(sevor_error_angle < -4095)
			{
				sevor_error_angle = 8195 - sevor_error_angle;
			}

			Chassis_Get_Control_Data.Vx = VX_SPEED*Chassis_Get_Control_Data.chassis_ctrl->rc.ch[0];
			Chassis_Get_Control_Data.Vy = VY_SPEED*Chassis_Get_Control_Data.chassis_ctrl->rc.ch[1];
			Chassis_Get_Control_Data.Wz = -18*sevor_error_angle;

			break;
		}
		
	}

	
	Chassis_Get_Control_Data.triggle = Chassis_Get_Control_Data.chassis_ctrl->rc.ch[4];
	
}


//获取底盘四个电机的转速并反结算出Vx Vy Wz
void Get_FeedBack(void)
{
	//反解算转速Vx
	Chassis_FeedBack_Data.Vx_FeedBack = -Chassis_FeedBack_Data.chassis_motor_data4->speed_rpm - Chassis_FeedBack_Data.chassis_motor_data3->speed_rpm + Chassis_FeedBack_Data.chassis_motor_data1->speed_rpm + Chassis_FeedBack_Data.chassis_motor_data2->speed_rpm;
	//反解算转速Vy
	Chassis_FeedBack_Data.Vy_FeedBack =  Chassis_FeedBack_Data.chassis_motor_data4->speed_rpm - Chassis_FeedBack_Data.chassis_motor_data3->speed_rpm + Chassis_FeedBack_Data.chassis_motor_data1->speed_rpm - Chassis_FeedBack_Data.chassis_motor_data2->speed_rpm;
	//反解算转速Wz
	Chassis_FeedBack_Data.Wz_FeedBack =  Chassis_FeedBack_Data.chassis_motor_data4->speed_rpm + Chassis_FeedBack_Data.chassis_motor_data3->speed_rpm + Chassis_FeedBack_Data.chassis_motor_data1->speed_rpm + Chassis_FeedBack_Data.chassis_motor_data2->speed_rpm;
	
	
	
	//计算出Vx
	Chassis_Set_Data.give_vx = VX_SPEED_DERECTION*PID_calc(&Vx_ref,Chassis_FeedBack_Data.Vx_FeedBack,Chassis_Get_Control_Data.Vx);
	//计算出Vy
	Chassis_Set_Data.give_vy = VY_SPEED_DERECTION*PID_calc(&Vy_ref,Chassis_FeedBack_Data.Vy_FeedBack,Chassis_Get_Control_Data.Vy);
	//计算出Wz
	Chassis_Set_Data.give_wz = WZ_SPEED_DERECTION*PID_calc(&Wz_ref,Chassis_FeedBack_Data.Wz_FeedBack,Chassis_Get_Control_Data.Wz);

}



//根据遥控器控制速度和电机反馈的速度进行pid计算
void Chassis_Get_Control_Speed(void)
{
	switch(Chassis_Get_Control_Data.chassis_ctrl->rc.s[1])
	{
		case WEEK:
		{
			Chassis_Speed.W1 = 0;
			Chassis_Speed.W2 = 0;
			Chassis_Speed.W3 = 0;
			Chassis_Speed.W4 = 0;
			break;
		}
		case ROTATING:
		{	
			Chassis_Speed.W1 =   Chassis_Set_Data.give_vx + Chassis_Set_Data.give_vy + Chassis_Set_Data.give_wz;
			Chassis_Speed.W2 = - Chassis_Set_Data.give_vx + Chassis_Set_Data.give_vy - Chassis_Set_Data.give_wz;
			Chassis_Speed.W3 =   Chassis_Set_Data.give_vx + Chassis_Set_Data.give_vy - Chassis_Set_Data.give_wz;
			Chassis_Speed.W4 = - Chassis_Set_Data.give_vx + Chassis_Set_Data.give_vy + Chassis_Set_Data.give_wz;
			break;
		}
		case SERVO:
		{
			Chassis_Speed.W1 =   Chassis_Set_Data.give_vx + Chassis_Set_Data.give_vy + Chassis_Set_Data.give_wz;
			Chassis_Speed.W2 = - Chassis_Set_Data.give_vx + Chassis_Set_Data.give_vy - Chassis_Set_Data.give_wz;
			Chassis_Speed.W3 =   Chassis_Set_Data.give_vx + Chassis_Set_Data.give_vy - Chassis_Set_Data.give_wz;
			Chassis_Speed.W4 = - Chassis_Set_Data.give_vx + Chassis_Set_Data.give_vy + Chassis_Set_Data.give_wz;
			break;
		}
	}
	if(Chassis_Get_Control_Data.chassis_ctrl->rc.s[1]!=WEEK)
	{
		if(Chassis_Get_Control_Data.chassis_ctrl->rc.ch[4] >600)
		{
			shoot_flag = 1;
			Chassis_Set_Data.give_trigge_current = -10000;
		}
		else Chassis_Set_Data.give_trigge_current = 0;
	}
}

void trrigler(void)
{
	if(shoot_flag == 1)
	{
//		error_angle=PID_calc(&trigger_angle,sevor_error_angle/MACHINE_ANGLE*360,SET_ANGLE);
//		Chassis_Set_Data.give_trigge_current = PID_calc(&trigger_speed,Chassis_FeedBack_Data.chassis_motor_data5->given_current,error_angle);
		Chassis_Set_Data.give_trigge_current = 10000;
	}
	else Chassis_Set_Data.give_trigge_current = 0;
	
	
	
	
}






void Chassis_Current_calc(void)
{
			
//			Chassis_Set_Data.give_motor1_current = MOTOR1*PID_calc(&wheel1,Chassis_FeedBack_Data.chassis_motor_data1->speed_rpm,Chassis_Speed.W1);
//			Chassis_Set_Data.give_motor2_current = MOTOR2*PID_calc(&wheel2,Chassis_FeedBack_Data.chassis_motor_data2->speed_rpm,Chassis_Speed.W2);
//			Chassis_Set_Data.give_motor3_current = MOTOR3*PID_calc(&wheel3,Chassis_FeedBack_Data.chassis_motor_data3->speed_rpm,Chassis_Speed.W3);
//			Chassis_Set_Data.give_motor4_current = MOTOR4*PID_calc(&wheel4,Chassis_FeedBack_Data.chassis_motor_data4->speed_rpm,Chassis_Speed.W4);
//			Chassis_Set_Data.give_triggle_current = MOTOR5*PID_calc(&triggler,Chassis_FeedBack_Data.chassis_motor_data5->speed_rpm,0);

}















