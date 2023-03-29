#include "Gimbal_task.h"
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

gimbal_feedback_typedef Gimbal_FeedBack_Data;
gimbal_typedef Gimbal_Set_Data;
gimbal_rc_control_typedef Gimbal_Get_Control_Data;
#define printf(...)			HAL_UART_Transmit(&huart2,\
                                           (uint8_t *)u1_buf,\
                                           sprintf((char*)u1_buf,__VA_ARGS__),\
                                           0xffff)			
uint8_t u1_buf[10];
																					 
																					 
																					 
void Gimbal_PID_init(void);
void Gimbal_init(gimbal_feedback_typedef *init);
void Gimbal_Control_init(void);
void Get_Control_Value(void);
void UpData_Feedback(void);
void Current_calc(void);

int16_t SEVER;
void Gimbal_task_Start(void const * argument)
{
  Gimbal_PID_init();
	Gimbal_init(&Gimbal_FeedBack_Data);
	Gimbal_Control_init();

  for(;;)
  {
		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_8);
		Get_Control_Value();
		UpData_Feedback();
		Current_calc();
		CAN_cmd_gimbal(Gimbal_Set_Data.give_yaw_current,Gimbal_Set_Data.give_pitch_current,0,0);//Gimbal_Set_Data.give_pitch_current
		CAN_cmd_shoot(Gimbal_Set_Data.give_shoot1_current,Gimbal_Set_Data.give_shoot2_current);
    osDelay(1);
  }
}

void Gimbal_PID_init(void)
{
	//yaw
	yaw_speed_pid[0] = YAW_SPEED_KP;
	yaw_position_pid[0] = YAW_POSITION_KP;
	
	yaw_speed_pid[1] = YAW_SPEED_KI;
	yaw_position_pid[1] = YAW_POSITION_KI;
	
	yaw_speed_pid[2] = YAW_SPEED_KD;
	yaw_position_pid[2] = YAW_POSITION_KD;
	
	
	//pitch
	pitch_speed_pid[0] = PITCH_SPEED_KP;
	pitch_position_pid[0] = PITCH_POSITION_KP;
	
	pitch_speed_pid[1] = PITCH_SPEED_KI;
	pitch_position_pid[1] = PITCH_POSITION_KI;
	
	pitch_speed_pid[2] = PITCH_SPEED_KD;
	pitch_position_pid[2] = PITCH_POSITION_KD;
	
	shoot1[0] = SHOOT1_KP;
	shoot1[1] = SHOOT1_KI;
	shoot1[2] = SHOOT1_KD;
	
	shoot2[0] = SHOOT2_KP;
	shoot2[1] = SHOOT2_KI;
	shoot2[2] = SHOOT2_KD;
	
	//pid初始化
	PID_init(&yaw_position_pid_ref,PID_POSITION,yaw_position_pid,MAX_PID_YAW_POSITION_OUT,MAX_PID_YAW_POSITION_IOUT);
	PID_init(&yaw_speed_pid_ref,PID_POSITION,yaw_speed_pid,MAX_PID_YAW_SPEED_OUT,MAX_PID_YAW_SPEED_IOUT);
	
	PID_init(&pitch_position_pid_ref,PID_POSITION,pitch_position_pid,MAX_PID_PITCH_POSITION_OUT,MAX_PID_PITCH_POSITION_IOUT);
	PID_init(&pitch_speed_pid_ref,PID_POSITION,pitch_speed_pid,MAX_PID_PITCH_SPEED_OUT,MAX_PID_PITCH_SPEED_IOUT);

	PID_init(&shoot1_pid_ref,PID_POSITION,shoot1,MAX_PID_SHOOT_OUT,MAX_PID_SHOOT_IOUT);
	PID_init(&shoot2_pid_ref,PID_POSITION,shoot2,MAX_PID_SHOOT_OUT,MAX_PID_SHOOT_IOUT);
}

void Gimbal_Control_init(void)
{
	Gimbal_Get_Control_Data.gimbal_ctrl = get_remote_control_point();
//	
//	Gimbal_Get_Control_Data.set_pitch_angle = imu.angle.yaw;
//	
//	Gimbal_Get_Control_Data.set_yaw_angle = imu.angle.pitch;
//	
	Gimbal_Get_Control_Data.set_yaw_angle=0;
	Gimbal_Get_Control_Data.set_pitch_angle = 0;
}


void Gimbal_init(gimbal_feedback_typedef *init)
{
	//获取电机数据指针
	init->yaw_motor_data		= get_yaw_gimbal_motor_measure_point();
	init->pitch_motor_data	= get_pitch_gimbal_motor_measure_point();
	
	init->shoot1_motor_data = get_shoot_motor_measure_point(0);
	init->shoot2_motor_data = get_shoot_motor_measure_point(1);
}


//等待加入卡尔曼数据融合
void UpData_Feedback(void)
{
//	//等待测试(经验卡尔曼数据融合)
//	Gimbal_FeedBack_Data.yaw_imu_ref_data.angle = imu.angle.yaw;
//	Gimbal_FeedBack_Data.yaw_imu_ref_data.rate = imu.rate.yaw ;
//	Gimbal_FeedBack_Data.pitch_imu_ref_data.angle = imu.angle.pitch;
//	Gimbal_FeedBack_Data.pitch_imu_ref_data.rate = imu.rate.pitch;
	
	//计算yaw error的值
	Gimbal_FeedBack_Data.motor.yaw_error_angle = Gimbal_Get_Control_Data.set_yaw_angle 	\
	- 2*360.0*Gimbal_FeedBack_Data.yaw_motor_data->ecd/MACHINE_ANGLE;
	//计算pitch error的值
	Gimbal_FeedBack_Data.motor.pitch_error_angle = Gimbal_Get_Control_Data.set_pitch_angle \
	- 2*360.0*Gimbal_FeedBack_Data.yaw_motor_data->ecd/MACHINE_ANGLE;
	
	
	
	//循环限幅
	if(Gimbal_FeedBack_Data.yaw_imu_error_data.angle > 180.0)
	{
	 Gimbal_FeedBack_Data.yaw_imu_error_data.angle = -360 + Gimbal_FeedBack_Data.yaw_imu_error_data.angle;
	}

	else if(Gimbal_FeedBack_Data.yaw_imu_error_data.angle < -180.0)
	{
		Gimbal_FeedBack_Data.yaw_imu_error_data.angle = 360 - Gimbal_FeedBack_Data.yaw_imu_error_data.angle;
	}
}

void Current_calc(void)
{
	switch(Gimbal_Get_Control_Data.gimbal_ctrl->rc.s[1])
	{ 
		case WEEK:
		{
			Gimbal_Set_Data.give_pitch_current = 0;
			Gimbal_Set_Data.give_yaw_current = 0;
			Gimbal_Set_Data.give_shoot1_current = 0;
			Gimbal_Set_Data.give_shoot2_current = 0;
			break;
		}
		
//		case ROTATING:
//		{
//			Gimbal_Set_Data.calc_yaw_angle = YAW_ANGLE_DERECTION*PID_calc(&yaw_position_pid_ref,Gimbal_FeedBack_Data.yaw_imu_error_data.angle,0);
//			Gimbal_Set_Data.give_yaw_current = YAW_CURRENT_DERECTION*PID_calc(&yaw_speed_pid_ref,Gimbal_FeedBack_Data.yaw_imu_ref_data.rate,Gimbal_Set_Data.calc_yaw_angle);
//							
//			Gimbal_Set_Data.calc_pitch_angle = PITCH_ANGLE_DERECTION*PID_calc(&pitch_position_pid_ref,Gimbal_FeedBack_Data.pitch_imu_error_data.angle,0);
//			Gimbal_Set_Data.give_pitch_current = PITCH_CURRENT_DERECTION*PID_calc(&pitch_speed_pid_ref,-Gimbal_FeedBack_Data.pitch_imu_ref_data.rate,Gimbal_Set_Data.calc_pitch_angle);
//			break;
//		}
		case SERVO:
		{	
			Gimbal_Set_Data.calc_yaw_angle = YAW_ANGLE_DERECTION*PID_calc(&yaw_position_pid_ref,\
			Gimbal_FeedBack_Data.motor.yaw_error_angle,0);
			Gimbal_Set_Data.give_yaw_current = YAW_CURRENT_DERECTION*PID_calc(&yaw_speed_pid_ref,\
			Gimbal_FeedBack_Data.motor.yaw_data.rate,Gimbal_Set_Data.calc_yaw_angle);
							
			Gimbal_Set_Data.calc_pitch_angle = PITCH_ANGLE_DERECTION*PID_calc(&pitch_position_pid_ref,\
			Gimbal_FeedBack_Data.motor.pitch_error_angle,0);
			Gimbal_Set_Data.give_pitch_current = PITCH_CURRENT_DERECTION*PID_calc(&pitch_speed_pid_ref,\
			-Gimbal_FeedBack_Data.motor.pitch_data.rate,Gimbal_Set_Data.calc_pitch_angle);
			

//			Gimbal_Set_Data.calc_yaw_angle = YAW_ANGLE_DERECTION*PID_calc(&yaw_position_pid_ref,Gimbal_FeedBack_Data.yaw_imu_error_data.angle,0);
//			Gimbal_Set_Data.give_yaw_current = YAW_CURRENT_DERECTION*PID_calc(&yaw_speed_pid_ref,Gimbal_FeedBack_Data.yaw_imu_ref_data.rate  ,  Gimbal_Set_Data.calc_yaw_angle);
//			
//			Gimbal_Set_Data.calc_pitch_angle = PITCH_ANGLE_DERECTION*PID_calc(&pitch_position_pid_ref,Gimbal_FeedBack_Data.pitch_imu_error_data.angle,0);
//			Gimbal_Set_Data.give_pitch_current = PITCH_CURRENT_DERECTION*PID_calc(&pitch_speed_pid_ref,-Gimbal_FeedBack_Data.pitch_imu_ref_data.rate,Gimbal_Set_Data.calc_pitch_angle);
			break;
		}
	}
	
	if(Gimbal_Get_Control_Data.gimbal_ctrl->rc.s[1] != WEEK)
	{
		switch(Gimbal_Get_Control_Data.gimbal_ctrl->rc.s[0])
		{
			case SHOOT_START_FLAG:
				Gimbal_Set_Data.give_shoot1_current = SHOOT1_CURRENT_DERECTION*PID_calc(&shoot1_pid_ref,Gimbal_FeedBack_Data.shoot1_motor_data->speed_rpm,SHOOT_SPEED);
				Gimbal_Set_Data.give_shoot2_current = SHOOT2_CURRENT_DERECTION*PID_calc(&shoot2_pid_ref,-Gimbal_FeedBack_Data.shoot2_motor_data->speed_rpm,SHOOT_SPEED);
				break;
			case SHOOT_STOP_FLAG:
				Gimbal_Set_Data.give_shoot1_current = PID_calc(&shoot1_pid_ref,Gimbal_FeedBack_Data.shoot1_motor_data->speed_rpm,SHOOT_STOP);
				Gimbal_Set_Data.give_shoot2_current = PID_calc(&shoot2_pid_ref,Gimbal_FeedBack_Data.shoot2_motor_data->speed_rpm,SHOOT_STOP);
			break;
		}
	}
}





void Get_Control_Value(void)
{ 
	Gimbal_Get_Control_Data.set_yaw_angle = Gimbal_Get_Control_Data.set_yaw_angle + YAW_SEN*rc_ctrl.rc.ch[2];
	Gimbal_Get_Control_Data.set_pitch_angle = Gimbal_Get_Control_Data.set_pitch_angle + PITCH_SEN*rc_ctrl.rc.ch[3];
//	
//	Gimbal_Get_Control_Data.set_yaw_angle = Gimbal_Get_Control_Data.set_yaw_angle;//转换为角度，单位：度
//	Gimbal_Get_Control_Data.set_pitch_angle = Gimbal_Get_Control_Data.set_pitch_angle;
	
	
	
	if(Gimbal_Get_Control_Data.set_yaw_angle > 180)
	{
		Gimbal_Get_Control_Data.set_yaw_angle -=360;
	}
	else if(Gimbal_Get_Control_Data.set_yaw_angle < -180)
	{
		Gimbal_Get_Control_Data.set_yaw_angle +=360;
	}
	
	
	
	if(Gimbal_Get_Control_Data.set_pitch_angle > MAX_PITCH_ANGLE)
	{
		Gimbal_Get_Control_Data.set_pitch_angle = MAX_PITCH_ANGLE;
	}
	else if(Gimbal_Get_Control_Data.set_pitch_angle < MIN_PITCH_ANGLE)
	{
		Gimbal_Get_Control_Data.set_pitch_angle = MIN_PITCH_ANGLE;
	}

	
}

//fp32* kalman_calc(uint8_t W1 ,uint8_t W2 , uint8_t num)
//{
//	data_W1[num] = W1 + 180;
//	data_W2[num] = W2 + 180;

//	fp32 K[2] = {0};
//	if(num == 99)
//	{
//		fp32 aver_W1 = 0;
//		fp32 aver_W2 = 0;
//		
//		for(uint8_t i = 0 ; i < 100 ; i++)
//		{ 
//			aver_W1 += data_W1[i];
//			aver_W2 += data_W2[i];
//		}
//		for(uint8_t i = 0; i < 100; i++)
//		{
//			K[0]+=(data_W1[i] - aver_W1)*(data_W1[i] - aver_W1);
//			K[1]+=(data_W2[i] - aver_W2)*(data_W2[i] - aver_W2);
//		}
//		K[0]/=100;
//		K[1]/=100;
//	}
//	return K;
//}
















