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

#define Delta_Yaw		-2.5
#define Delta_Pitch 0.010
#define AIMBOT_LOST_TIME 1000


uint32_t Auto_Work_Time;
double K1 = 0,K2 = 0;
fp32 Start_Yaw = 0;
fp32 Start_Pitch = 0;
uint8_t AThis = 0, ALast = 1;
uint32_t Aimbot_Lost_Time = AIMBOT_LOST_TIME;
first_order_filter_type_t pitch_aimbot_filter;
fp32 pitch_aimbot_filter_param = 0.10f;

void Auto_work_init(void);
void Auto_Move(void);
void Auto_Work_Detect(void);

void Auto_work_init(void)
{
	//获取pitch轴角度更新参数
	K2 = (PITCH_MIN_ANGLE + PITCH_MAX_ANGLE)/2;
	K1 = PITCH_MAX_ANGLE - K2;
	Start_Pitch = acos(Gimbal.Imu.PitchAngle / 360.0);
	Auto_Work_Time = 0;
	first_order_filter_init(&pitch_aimbot_filter, 1000, &pitch_aimbot_filter_param);
}

void Auto_Move(void)
{
	GetAimbotCommand(&My_AimbotCommand);
	if(Gimbal.StateMachine == GM_MATCH)
	{
		//当当前处于视觉控制或者自動控制模式时
		if(Gimbal.ControlMode == GM_AIMBOT_OPERATE || Gimbal.ControlMode == GM_AUTO_OPERATE)
		{
			//当视觉失去目标时
			if(((My_AimbotCommand.State & AIMBOT_TARGET_INSIDE_OFFSET)) == 0 
				&& Aimbot_Lost_Time == 0)
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
			//進入丟失狀態，此時不進行控制
			else if((((My_AimbotCommand.State & AIMBOT_TARGET_INSIDE_OFFSET)) == 0 && Aimbot_Lost_Time > 0))
			{
				Aimbot_Lost_Time--;
				Gimbal.Command.Yaw = Gimbal.Imu.YawAngle;
				Gimbal.Command.Pitch = Gimbal.Imu.PitchAngle;
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
		CAN_RC_TO_RC(&RC_CTRL,&Can_RC);
		Gimbal.Command.Yaw -= RC_CTRL.rc.ch[2] / 660.0f * 0.25;
		Gimbal.Command.Pitch -= RC_CTRL.rc.ch[3] / 660.0f * 0.25;
		Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
		Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
		Gimbal.Output.Yaw = cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle, Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
		Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);

		if(((My_AimbotCommand.State & AIMBOT_TARGET_INSIDE_OFFSET) == 1)
			&&(Offline.AimbotStateNode == DEVICE_ONLINE)
			&&(Offline.AimbotDataNode == DEVICE_ONLINE))
		{
			AThis = 1;
			Auto_Work_Detect();
			Gimbal.Command.Yaw   = LoopFifoFp32_read(&Gimbal.ImuBuffer.YawLoopPointer, (GetSystemTimer() - My_AimbotCommand.CommandTimer)) + My_AimbotCommand.YawRelativeAngle;
			Gimbal.Command.Pitch   = LoopFifoFp32_read(&Gimbal.ImuBuffer.PitchLoopPointer, (GetSystemTimer() - My_AimbotCommand.CommandTimer)) + My_AimbotCommand.PitchRelativeAngle;
			
//			fp32 pitch_command_temp = LoopFifoFp32_read(&Gimbal.ImuBuffer.PitchLoopPointer, (GetSystemTimer() - My_AimbotCommand.CommandTimer)) + My_AimbotCommand.PitchRelativeAngle;
//			first_order_filter_cali(&pitch_aimbot_filter, pitch_command_temp);
//			Gimbal.Command.Pitch = pitch_aimbot_filter.out;
			
			Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
			Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
			Gimbal.Output.Yaw = cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle, Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
			Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);
			Aimbot_Lost_Time = AIMBOT_LOST_TIME;
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






























