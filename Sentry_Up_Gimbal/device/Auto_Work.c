#include "stdio.h"
#include "Auto_Work.h"
#include "main.h"
#include "AimbotCan.h"
#include "CalculateThread.h"
#include "InterruptService.h"
#include "arm_math.h"
#include "Setting.h"
#include "user_lib.h"
#include "pid.h"
#include "loop_fifo.h"
#include "Can_Remote.h"



AimbotCommand_t  My_AimbotCommand;
RC_ctrl_t        RC_CTRL;

void GetAimbotCommand(AimbotCommand_t *Aimbot);
extern Gimbal_t Gimbal;

extern OfflineMonitor_t Offline;

#define Delta_Yaw		 -1
#define Delta_Yaw1		 0.003
#define Delta_Pitch		 0.015
#define AIMBOT_LOST_TIME 700

uint32_t Auto_Work_Time;
double K1 = 0,K2 = 0,K3 = 0,K4 = 0;
fp32 Start_Yaw = 0;
fp32 Start_Pitch = 0;
uint8_t AThis = 0, ALast = 1;
uint32_t Aimbot_Lost_Time = AIMBOT_LOST_TIME;
first_order_filter_type_t pitch_aimbot_filter;
fp32 pitch_aimbot_filter_param = 0.10f;

fp32 PITCH_MAX_ANGLE_TEST = PITCH_MAX_ANGLE - 10;
fp32 PITCH_MIN_ANGLE_TEST = PITCH_MIN_ANGLE + 17;

fp32 YAW_MAX_ANGLE_TEST = 18;
fp32 YAW_MIN_ANGLE_TEST = -85;

void Auto_work_init(void);
void Auto_Move(void);
void Auto_Work_Detect(void);

void Auto_work_init(void)
{
	//获取pitch轴角度更新参数
	K2 = (PITCH_MIN_ANGLE_TEST + PITCH_MAX_ANGLE_TEST)/2;
	K1 = PITCH_MAX_ANGLE_TEST - K2;
	
	K4 = (YAW_MIN_ANGLE_TEST + YAW_MAX_ANGLE_TEST)/2;
	K3 =  YAW_MAX_ANGLE_TEST - K4;
	
	Start_Pitch = acos(Gimbal.Imu.PitchAngle / 360.0);
	Start_Yaw = acos(Gimbal.MotorMeasure.GimbalMotor.YawMotorAngle / 360.0);
	Auto_Work_Time = 0;
	
	
	
	
//	first_order_filter_init(&pitch_aimbot_filter, 1000, &pitch_aimbot_filter_param);
}

void Auto_Move(void)
{
	GetAimbotCommand(&My_AimbotCommand);
	if(Gimbal.StateMachine == GM_MATCH)
	{
		//当当前处于视觉控制或者自動控制模式时
		if(Gimbal.ControlMode == GM_AIMBOT_OPERATE 
		   || Gimbal.ControlMode == GM_AUTO_OPERATE)
		{
			//当视觉失去目标,前哨战被摧毁时
			if(((My_AimbotCommand.State & AIMBOT_TARGET_INSIDE_OFFSET)) == 0 
				&& Aimbot_Lost_Time == 0
				&& Game_State.Defend_Mode == Sentry_Defend)
			{
				AThis = 0;
				Auto_Work_Detect();
				
				Gimbal.Command.Yaw = Gimbal.Imu.YawAngle + Delta_Yaw;
				Gimbal.Command.Pitch = K1*cos(Start_Pitch + Delta_Pitch*Auto_Work_Time) + K2;
				Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
				Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
				Gimbal.Output.Yaw = cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle, Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
				Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);
				

			}
			//当视觉失去目标,前哨战未被摧毁时
			else if(((My_AimbotCommand.State & AIMBOT_TARGET_INSIDE_OFFSET)) == 0 
				     && Aimbot_Lost_Time == 0
					 && Game_State.Defend_Mode == Sentry_Escape)
			{
				AThis = 0;
				Auto_Work_Detect();

				Gimbal.Command.Yaw = K3*cos(Start_Yaw + Delta_Yaw1*Auto_Work_Time) + K4;
				Gimbal.Command.Pitch = K1*cos(Start_Pitch + Delta_Pitch*Auto_Work_Time) + K2;
				Gimbal.Command.Yaw =   fp32_constrain(Gimbal.Command.Yaw,YAW_MIN_ANGLE_TEST,YAW_MAX_ANGLE_TEST);
				Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
				Gimbal.Output.Yaw = cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.MotorMeasure.GimbalMotor.YawMotorAngle, \
													  Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
				Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);
				
				
				
			}
			//進入丟失狀態，此時不進行控制
			else if((((My_AimbotCommand.State & AIMBOT_TARGET_INSIDE_OFFSET)) == 0 && Aimbot_Lost_Time > 0))
			{
				Aimbot_Lost_Time--;
				Gimbal.Command.Yaw = Gimbal.Command.Yaw;
				Gimbal.Command.Pitch = Gimbal.Command.Pitch;
				Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
				Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
				Gimbal.Output.Yaw = cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle, Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
				Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);	
				pitch_aimbot_filter.out = Gimbal.Command.Pitch;
			}
			//开始视觉跟随控制
			else if((((My_AimbotCommand.State & AIMBOT_TARGET_INSIDE_OFFSET)) == 1)
					 &&(Offline.AimbotStateNode == DEVICE_ONLINE)
					 &&(Offline.AimbotDataNode == DEVICE_ONLINE))
			{
				AThis = 1;
				Auto_Work_Detect();
				Gimbal.Command.Yaw   = LoopFifoFp32_read(&Gimbal.ImuBuffer.YawLoopPointer, (GetSystemTimer() - My_AimbotCommand.CommandTimer)) + My_AimbotCommand.YawRelativeAngle;
				Gimbal.Command.Pitch   = LoopFifoFp32_read(&Gimbal.ImuBuffer.PitchLoopPointer, (GetSystemTimer() - My_AimbotCommand.CommandTimer)) + My_AimbotCommand.PitchRelativeAngle;
			
//				fp32 pitch_command_temp = LoopFifoFp32_read(&Gimbal.ImuBuffer.PitchLoopPointer, (GetSystemTimer() - Aimbot.CommandTimer)) + Aimbot.PitchRelativeAngle;
//				first_order_filter_cali(&pitch_aimbot_filter, pitch_command_temp);
//				Gimbal.Command.Pitch = pitch_aimbot_filter.out;
		
				Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
				Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
				Gimbal.Output.Yaw = cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle, Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
				Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);
				Aimbot_Lost_Time = AIMBOT_LOST_TIME;
			}
		}
	}
	else if(Gimbal.StateMachine == GM_TEST)
	{
		if((My_AimbotCommand.State & AIMBOT_TARGET_INSIDE_OFFSET) == 0)
		{
			CAN_RC_TO_RC(&RC_CTRL,&Can_RC);
			Gimbal.Command.Yaw -= RC_CTRL.rc.ch[2] / 660.0f * 0.25;
			Gimbal.Command.Pitch -= RC_CTRL.rc.ch[3] / 660.0f * 0.25;
			Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
			Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
			Gimbal.Output.Yaw = cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle, Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
			Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);
		}
		else if((((My_AimbotCommand.State & AIMBOT_TARGET_INSIDE_OFFSET)) == 1)
					 &&(Offline.AimbotStateNode == DEVICE_ONLINE)
					 &&(Offline.AimbotDataNode == DEVICE_ONLINE))
		{
			AThis = 1;
			Auto_Work_Detect();
			Gimbal.Command.Yaw   = LoopFifoFp32_read(&Gimbal.ImuBuffer.YawLoopPointer, (GetSystemTimer() - My_AimbotCommand.CommandTimer)) + My_AimbotCommand.YawRelativeAngle;
			Gimbal.Command.Pitch   = LoopFifoFp32_read(&Gimbal.ImuBuffer.PitchLoopPointer, (GetSystemTimer() - My_AimbotCommand.CommandTimer)) + My_AimbotCommand.PitchRelativeAngle;
//				fp32 pitch_command_temp = LoopFifoFp32_read(&Gimbal.ImuBuffer.PitchLoopPointer, (GetSystemTimer() - Aimbot.CommandTimer)) + Aimbot.PitchRelativeAngle;
//				first_order_filter_cali(&pitch_aimbot_filter, pitch_command_temp);
//				Gimbal.Command.Pitch = pitch_aimbot_filter.out;
			
   		    Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
			Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
			Gimbal.Output.Yaw = cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle, Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
			Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);
		}
	}
}

void Auto_Work_Detect(void)
{
	if(ALast != AThis)
	{
		if(ALast == 1 && AThis == 0)
		{
			Auto_work_init();
		}
			ALast = AThis;
	}
}






























