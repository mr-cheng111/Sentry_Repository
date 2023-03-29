#include "CalculateThread.h"
#include "AttitudeThread.h"
#include "InterruptService.h"
#include "Remote.h"
#include "Can_Remote.h"
#include "AimbotCan.h"
#include "user_lib.h"
#include "pid.h"
#include "Motor.h"
#include "RefereeCan.h"
#include "Auto_Work.h"
#include "bsp_can.h"
#include "loop_fifo.h"
#include "cmsis_os.h"
#include <string.h>
#include "Setting.h"
#include "Usart_HMI.h"
#include PARAMETER_FILE
#include "CanPacket.h"
#include "CalculateThread.h"



Gimbal_t                Gimbal;
RC_ctrl_t               Remote;
AimbotCommand_t         Aimbot;
OfflineMonitor_t        Offline;
RefereeInformation_t    Referee;

void GimbalStateMachineUpdate(void);
void GimbalControlModeUpdate(void);
void GimbalFireModeUpdate(void);
void SetGimbalDisable(void);
void GimbalPIDUpdate(void);
void RotorPIDUpdate(void);
void AmmoPIDUpdate(void);
void GimbalMeasureUpdate(void);
void GimbalCommandUpdate(void);
void RotorCommandUpdate(void);
void AmmoCommandUpdate(void);
void DebugLEDShow(void);
void RefereeHeat_init(void);

fp32 LimitNormalization(fp32 input);

void CalculateThread(void const * pvParameters)
{
    osDelay(500);
    PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_30MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
    PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_30MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
    LoopFifoFp32_init(&Gimbal.ImuBuffer.YawLoopPointer, Gimbal.ImuBuffer.YawAddress, 64);
    LoopFifoFp32_init(&Gimbal.ImuBuffer.PitchLoopPointer, Gimbal.ImuBuffer.PitchAddress, 64);

	
    while(1)
    {
		CAN_RC_TO_RC(&Remote,&Can_RC);
        GetAimbotCommand(&Aimbot);
        GetRefereeInformation(&Referee);
		RefereeHeat_init();
        DeviceOfflineMonitorUpdate(&Offline);

        LoopFifoFp32_push(&Gimbal.ImuBuffer.YawLoopPointer, Gimbal.Imu.YawAngle);
        LoopFifoFp32_push(&Gimbal.ImuBuffer.PitchLoopPointer, Gimbal.Imu.PitchAngle);

        GimbalStateMachineUpdate();
        GimbalControlModeUpdate();
        GimbalFireModeUpdate();
        GimbalPIDUpdate();
        RotorPIDUpdate();
        AmmoPIDUpdate();

        GimbalMeasureUpdate();
        GimbalCommandUpdate();
        RotorCommandUpdate();
        AmmoCommandUpdate();
        
        GimbalMotorControl( Gimbal.Output.Yaw * YAW_MOTOR_DIRECTION,
                            Gimbal.Output.Pitch * PITCH_MOTOR_DIRECTION, 
                            Gimbal.Output.Rotor, 
                            Gimbal.Output.AmmoLeft, 
                            Gimbal.Output.AmmoRight
                          );
        DebugLEDShow();
        osDelay(1);
    }
}

uint32_t    gimbal_init_countdown = 0;          //  云台初始化倒计时器
uint32_t    gimbal_fire_countdown = 0;          //  云台射击拨盘转动倒计时器
uint32_t    gimbal_cooling_countdown = 0;       //  云台冷却倒计时器
uint32_t    gimbal_lagging_counter = 0;         //  云台堵转计数器
uint32_t    gimbal_reverse_countdown = 0;       //  云台拨盘反转倒计时器
void GimbalStateMachineUpdate(void)
{
	//云台状态机
	//右上，比赛模式
	//左下右中，调试模式
	//右下，无力
	if(Remote.rc.s[0] == RC_SW_UP)
	{
		if (Gimbal.StateMachine == GM_NO_FORCE)
		{
			Gimbal.StateMachine = GM_INIT;
			gimbal_init_countdown = 50;
		}
		else if (Gimbal.StateMachine == GM_INIT)
		{
			if (gimbal_init_countdown > 0)
			{
				gimbal_init_countdown--;
			}
			else
			{
				Gimbal.StateMachine = GM_MATCH;
			}
		}
		else
		{
			Gimbal.StateMachine = GM_MATCH;
		}
	}
#ifdef SENTRY7_UP_GIMBAL
	else if (Remote.rc.s[1] == RC_SW_UP)
	{
		switch (Remote.rc.s[0])
			{
				// 右拨杆打到中间，云台复位后进入调试模式
				case RC_SW_MID:
				{
					if (Gimbal.StateMachine == GM_NO_FORCE)
					{
						Gimbal.StateMachine = GM_INIT;
						gimbal_init_countdown = 50;
					}
					else if (Gimbal.StateMachine == GM_INIT)
					{
						if (gimbal_init_countdown > 0)
						{
							gimbal_init_countdown--;
						}
						else
						{
							Gimbal.StateMachine = GM_TEST;
						}
					}
					else
					{
						Gimbal.StateMachine = GM_TEST;
					}break;
				}
				// 右拨杆打到最下，或遥控器数据出错，云台进入无力模式
				case RC_SW_DOWN:
				{
					if (Gimbal.StateMachine != GM_NO_FORCE)
					{
						Gimbal.StateMachine = GM_NO_FORCE;
					}break;
				}
			}
	}
		
#else
	else if (Remote.rc.s[1] == RC_SW_DOWN)
	{
		switch (Remote.rc.s[0])
			{
				// 右拨杆打到中间，云台复位后进入调试模式
				case RC_SW_MID:
				{
					if (Gimbal.StateMachine == GM_NO_FORCE)
					{
						Gimbal.StateMachine = GM_INIT;
						gimbal_init_countdown = 50;
					}
					else if (Gimbal.StateMachine == GM_INIT)
					{
						if (gimbal_init_countdown > 0)
						{
							gimbal_init_countdown--;
						}
						else
						{
							Gimbal.StateMachine = GM_TEST;
						}
					}
					else
					{
						Gimbal.StateMachine = GM_TEST;
					}break;
				}
				// 右拨杆打到最下，或遥控器数据出错，云台进入无力模式
				case RC_SW_DOWN:
				{
					if (Gimbal.StateMachine != GM_NO_FORCE)
					{
						Gimbal.StateMachine = GM_NO_FORCE;
					}break;
				}
			}	
	}
	
#endif

	else if (Gimbal.StateMachine != GM_NO_FORCE)
	{
		Gimbal.StateMachine = GM_NO_FORCE;
	}
	// 右拨杆打到最上，云台复位后进入比赛模式，该模式下开摩擦轮

}

void SetGimbalDisable(void)
{
    Gimbal.StateMachine = GM_NO_FORCE;
    Gimbal.ControlMode = GM_NO_CONTROL;
    Gimbal.FireMode = GM_FIRE_UNABLE;
}
void GimbalControlModeUpdate(void)
{
    // 比赛模式下
    if(Gimbal.StateMachine == GM_MATCH)
	{
        //视觉发现目标，进入自瞄控制
        if((Offline.AimbotStateNode == DEVICE_ONLINE)
		  &&(Offline.AimbotDataNode == DEVICE_ONLINE)
          &&(Aimbot.State & AIMBOT_TARGET_INSIDE_OFFSET))
		{
			Gimbal.ControlMode = GM_AIMBOT_OPERATE;
        }
        else
		{
            Gimbal.ControlMode = GM_AUTO_OPERATE;
        }
    }
    else if (Gimbal.StateMachine == GM_TEST)
	{
		if(Aimbot.State & AIMBOT_TARGET_INSIDE_OFFSET
		&&(Offline.AimbotStateNode == DEVICE_ONLINE)
		&&(Offline.AimbotDataNode == DEVICE_ONLINE))
		{
			Gimbal.ControlMode = GM_AIMBOT_OPERATE;
		}
		else
		{
			Gimbal.ControlMode = GM_MANUAL_OPERATE;
		}
	}
    else if (Gimbal.StateMachine == GM_INIT)
	{
        Gimbal.ControlMode = GM_RESET_POSITION;
    }
    else
	{
        Gimbal.ControlMode = GM_NO_CONTROL;
    }
}
/*
  状态量：
	1.Game_Start_State;
	2.Defend_Mode;
	3.Point_Fire_Mode;
*/
uint16_t Ammo_Counter = 0;
uint8_t  Ammo_Flag = 0;
void GimbalFireModeUpdate(void)
{
	/*测试模式下，只允许点射*/
	if(Remote.rc.ch[4] == -660)
	{
		Ammo_Counter++;
	}
	if(Ammo_Counter < 1000)
	{
		Ammo_Flag = 0;
	}
	else if(Ammo_Counter < 2000)
	{
		Ammo_Flag = 1;
	}
	else
	{
		Ammo_Counter = 0;
	}
	
#ifdef SENTRY_UP_GIMBAL
	if ( Ammo_Flag && Gimbal.StateMachine != GM_MATCH && Remote.rc.s[1] == RC_SW_UP)
	{
		if (Gimbal.FireMode == GM_FIRE_UNABLE)
		{
			Gimbal.FireMode = GM_FIRE_READY;
		}
		//  正常射击模式的状态机 : "就绪->射击->冷却->就绪->......"
		if (Gimbal.FireMode == GM_FIRE_READY)
		{
			if(Remote.rc.ch[4] == 660)						
			{
				Gimbal.FireMode = GM_FIRE_BUSY;
				gimbal_fire_countdown = POINTFIRE_ROTOR_TIMESET_BUSY_TWO;
				Gimbal.Referee.CurrentHeat += 10;
			}
		}
		else if (Gimbal.FireMode == GM_FIRE_BUSY)
		{
			if (gimbal_fire_countdown > 0)
			{
				gimbal_fire_countdown--;
			}
			else
			{
				Gimbal.FireMode = GM_FIRE_COOLING;
				gimbal_cooling_countdown = POINTFIRE_ROTOR_TIMESET_COOLING_TWO;
			}
		}
		else if (Gimbal.FireMode == GM_FIRE_COOLING)
		{
			if  (gimbal_cooling_countdown > 0)
			{
				gimbal_cooling_countdown--;
			}
			else
			{
				Gimbal.FireMode = GM_FIRE_READY;
			}
		}
		//  异常射击模式的状态机，用于反堵转
		else if (Gimbal.FireMode == GM_FIRE_LAGGING)
		{
			if (gimbal_reverse_countdown > 0)
			{
				gimbal_reverse_countdown --;
			}
			else
			{
				Gimbal.FireMode = GM_FIRE_READY;
			}
		}
		//  堵转计数
		if ((Gimbal.FireMode == GM_FIRE_BUSY) && (Gimbal.MotorMeasure.ShootMotor.RotorMotorSpeed < 100))
		{
			gimbal_lagging_counter++;
		}
		else
		{
			gimbal_lagging_counter = 0;
		}
		//  触发反堵转状态机
		if (gimbal_lagging_counter > ROTOR_LAGGING_COUNTER_MAX)
		{
			gimbal_lagging_counter = 0;
			gimbal_reverse_countdown = ROTOR_TIMESET_RESERVE;
			Gimbal.FireMode = GM_FIRE_LAGGING;
		}
	}
	else if(Ammo_Flag == 0 && Gimbal.StateMachine != GM_MATCH)
	{
		Gimbal.FireMode = GM_FIRE_UNABLE;
	}
#else 
	if ( Ammo_Flag && Gimbal.StateMachine != GM_MATCH && Remote.rc.s[1] == RC_SW_DOWN)
	{
		if (Gimbal.FireMode == GM_FIRE_UNABLE)
		{
			Gimbal.FireMode = GM_FIRE_READY;
		}
		//  正常射击模式的状态机 : "就绪->射击->冷却->就绪->......"
		if (Gimbal.FireMode == GM_FIRE_READY)
		{
			if(Remote.rc.ch[4] == 660)						
			{
				Gimbal.FireMode = GM_FIRE_BUSY;
				gimbal_fire_countdown = POINTFIRE_ROTOR_TIMESET_BUSY_TWO;
				Gimbal.Referee.CurrentHeat += 10;
			}
		}
		else if (Gimbal.FireMode == GM_FIRE_BUSY)
		{
			if (gimbal_fire_countdown > 0)
			{
				gimbal_fire_countdown--;
			}
			else
			{
				Gimbal.FireMode = GM_FIRE_COOLING;
				gimbal_cooling_countdown = POINTFIRE_ROTOR_TIMESET_COOLING_TWO;
			}
		}
		else if (Gimbal.FireMode == GM_FIRE_COOLING)
		{
			if  (gimbal_cooling_countdown > 0)
			{
				gimbal_cooling_countdown--;
			}
			else
			{
				Gimbal.FireMode = GM_FIRE_READY;
			}
		}
		//  异常射击模式的状态机，用于反堵转
		else if (Gimbal.FireMode == GM_FIRE_LAGGING)
		{
			if (gimbal_reverse_countdown > 0)
			{
				gimbal_reverse_countdown --;
			}
			else
			{
				Gimbal.FireMode = GM_FIRE_READY;
			}
		}
		//  堵转计数
		if ((Gimbal.FireMode == GM_FIRE_BUSY) && (Gimbal.MotorMeasure.ShootMotor.RotorMotorSpeed < 100))
		{
			gimbal_lagging_counter++;
		}
		else
		{
			gimbal_lagging_counter = 0;
		}
		//  触发反堵转状态机
		if (gimbal_lagging_counter > ROTOR_LAGGING_COUNTER_MAX)
		{
			gimbal_lagging_counter = 0;
			gimbal_reverse_countdown = ROTOR_TIMESET_RESERVE;
			Gimbal.FireMode = GM_FIRE_LAGGING;
		}
	}
	else if(Ammo_Flag == 0 && Gimbal.StateMachine != GM_MATCH)
	{
		Gimbal.FireMode = GM_FIRE_UNABLE;
	}
#endif
	//比赛模式，会判断是否进入比赛模式，在未进入比赛模式前，将无法打子弹
	else if (Gimbal.StateMachine == GM_MATCH)
	{
		if (Gimbal.FireMode == GM_FIRE_UNABLE)
		{
			Gimbal.FireMode = GM_FIRE_READY;
		}
		//  正常射击模式的状态机 : "就绪->射击->冷却->就绪->......"
		if (Gimbal.FireMode == GM_FIRE_READY)
		{
			//视觉请求发弹，并且视觉未掉线
			if(((Aimbot.State & AIMBOT_SHOOT_REQUEST_OFFSET))
				&&(Offline.AimbotStateNode == DEVICE_ONLINE)
//				&&(Gimbal.Referee.CurrentHeat < MAX_HEAT)
				&&(Game_State.Game_Start_State))//判断是否进入比赛模式，在未进入比赛模式前，将无法打子弹,不打工程
			{
				Gimbal.FireMode = GM_FIRE_BUSY;
				
				//此处需要加上射速设置
				switch(Aimbot.Target)
				{
					case 1: gimbal_fire_countdown = BUSTFIRE_ROTOR_TIMESET_BUSY;break;
					case 2: gimbal_fire_countdown = POINTFIRE_ROTOR_TIMESET_BUSY_ONE;break;
					case 3: gimbal_fire_countdown = POINTFIRE_ROTOR_TIMESET_BUSY_TWO;break;
					default: gimbal_fire_countdown = POINTFIRE_ROTOR_TIMESET_BUSY_ONE;break;
				}
				Gimbal.Referee.CurrentHeat += 10;
			}
		}
		else if (Gimbal.FireMode == GM_FIRE_BUSY)
		{
			if (gimbal_fire_countdown > 0)
			{
				gimbal_fire_countdown--;
			}
			else
			{
				Gimbal.FireMode = GM_FIRE_COOLING;
				switch(Game_State.Point_Fire_Mode)
				{
					case Sentry_Brust_Fire: gimbal_cooling_countdown = BUSTFIRE_ROTOR_TIMESET_COOLING;break;
					case Sentry_Point_Fire: gimbal_cooling_countdown = POINTFIRE_ROTOR_TIMESET_COOLING_ONE;break;
					default: gimbal_cooling_countdown = POINTFIRE_ROTOR_TIMESET_COOLING_ONE;break;
				}
			}
		}
		else if (Gimbal.FireMode == GM_FIRE_COOLING)
		{
			if (gimbal_cooling_countdown > 0)
			{
				gimbal_cooling_countdown--;
			}
			else
			{
				Gimbal.FireMode = GM_FIRE_READY;
			}
		}
		//  异常射击模式的状态机，用于反堵转
		else if (Gimbal.FireMode == GM_FIRE_LAGGING)
		{
			if (gimbal_reverse_countdown > 0)
			{
				gimbal_reverse_countdown --;
			}
			else
			{
				Gimbal.FireMode = GM_FIRE_READY;
			}
		}
		//  堵转计数
		if ((Gimbal.FireMode == GM_FIRE_BUSY) && (Gimbal.MotorMeasure.ShootMotor.RotorMotorSpeed < 100))
		{
			gimbal_lagging_counter++;
		}
		else
		{
			gimbal_lagging_counter = 0;
		}
		//  触发反堵转状态机
		if (gimbal_lagging_counter > ROTOR_LAGGING_COUNTER_MAX)
		{
			gimbal_lagging_counter = 0;
			gimbal_reverse_countdown = ROTOR_TIMESET_RESERVE;
			Gimbal.FireMode = GM_FIRE_LAGGING;
		}
	}
	else
	{
		Gimbal.FireMode = GM_FIRE_UNABLE;
	}
}

GimbalControlMode_e CMthis = GM_NO_CONTROL;
GimbalControlMode_e CMlast = GM_NO_CONTROL;
void GimbalPIDUpdate(void)
{
    CMthis = Gimbal.ControlMode;
    
    if (CMthis == CMlast)
	{
        return;
    }
    
    if (CMthis == GM_MANUAL_OPERATE)
	{
        cascade_PID_init(   &Gimbal.Pid.Yaw, 
                            YAW_ANGLE_MANUAL_OPERATE, 
                            YAW_SPEED_MANUAL_OPERATE, 
                            YAW_MAX_SPEED, 
                            YAW_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
        cascade_PID_init(   &Gimbal.Pid.Pitch, 
                            PITCH_ANGLE_MANUAL_OPERATE, 
                            PITCH_SPEED_MANUAL_OPERATE, 
                            PITCH_MAX_SPEED, 
                            PITCH_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
    }
    else if (CMthis == GM_AIMBOT_OPERATE)
	{
        cascade_PID_init(   &Gimbal.Pid.Yaw, 
                            YAW_ANGLE_AIMBOT_OPERATE, 
                            YAW_SPEED_AIMBOT_OPERATE, 
                            YAW_MAX_SPEED, 
                            YAW_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
        cascade_PID_init(   &Gimbal.Pid.Pitch, 
                            PITCH_ANGLE_AIMBOT_OPERATE, 
                            PITCH_SPEED_AIMBOT_OPERATE, 
                            PITCH_MAX_SPEED, 
                            PITCH_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
    }
    else if (CMthis == GM_AUTO_OPERATE)
	{
        cascade_PID_init(   &Gimbal.Pid.Yaw, 
                            YAW_ANGLE_AIMBOT_OPERATE, 
                            YAW_SPEED_AIMBOT_OPERATE, 
                            YAW_MAX_SPEED, 
                            YAW_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
        cascade_PID_init(   &Gimbal.Pid.Pitch, 
                            PITCH_ANGLE_AIMBOT_OPERATE, 
                            PITCH_SPEED_AIMBOT_OPERATE, 
                            PITCH_MAX_SPEED, 
                            PITCH_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
    }

    else
	{
        cascade_PID_init(   &Gimbal.Pid.Yaw, 
                            YAW_ANGLE_NO_FORCE, 
                            YAW_SPEED_NO_FORCE, 
                            YAW_MAX_SPEED, 
                            YAW_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
        cascade_PID_init(   &Gimbal.Pid.Pitch, 
                            PITCH_ANGLE_NO_FORCE, 
                            PITCH_SPEED_NO_FORCE, 
                            PITCH_MAX_SPEED, 
                            PITCH_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
    }	
    CMlast = CMthis;
}

GimbalFireMode_e FMthis = GM_FIRE_UNABLE;
GimbalFireMode_e FMlast = GM_FIRE_UNABLE;
void RotorPIDUpdate(void)
{
    FMthis = Gimbal.FireMode;
    if (FMthis == FMlast)
	{
        return;
    }
    if ((FMthis == GM_FIRE_READY)  ||  (FMthis == GM_FIRE_COOLING))
	{
        PID_init(&Gimbal.Pid.Rotor, PID_POSITION, ROTOR_STOP, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
    }
    else if (FMthis == GM_FIRE_BUSY)
	{
        PID_init(&Gimbal.Pid.Rotor, PID_POSITION, ROTOR_FORWARD, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
    }
    else if (FMthis == GM_FIRE_LAGGING)
	{
        PID_init(&Gimbal.Pid.Rotor, PID_POSITION, ROTOR_BACK, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
    }
    else
	{
        PID_init(&Gimbal.Pid.Rotor, PID_POSITION, ROTOR_UNABLE, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
    }
    
    FMlast = FMthis;
}
uint8_t MSthis = 0;
uint8_t MSlast = 0;

extern float Lkp;
extern float Rkp;
void AmmoPIDUpdate(void)
{
    MSthis = Gimbal.Referee.MaxSpeed;
    AMMO_LEFT_SPEED_30MS[0]  = Lkp;
	  AMMO_RIGHT_SPEED_30MS[0] = Rkp;
    if (MSthis != MSlast)
	{
        switch (MSthis)
		{
            case 30:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_30MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_30MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            default:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_30MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_30MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
        }
    }
    MSlast = MSthis;
}

void GimbalMeasureUpdate(void)
{
    GimbalMotorMeasureUpdate(&Gimbal.MotorMeasure.GimbalMotor);
    ShootMotorMeasureUpdate(&Gimbal.MotorMeasure.ShootMotor);
    GimbalEulerSystemMeasureUpdate(&Gimbal.Imu);
}

void GimbalCommandUpdate(void)
{
    if (Gimbal.ControlMode == GM_MANUAL_OPERATE)
	{
		Gimbal.Command.Yaw -= Remote.rc.ch[2] / 660.0f * YAW_REMOTE_SENS;
        Gimbal.Command.Pitch -= Remote.rc.ch[3] / 660.0f * PITCH_REMOTE_SENS;
        Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
        Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
        Gimbal.Output.Yaw = cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle, Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
        Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);
    }
	else if (Gimbal.ControlMode == GM_AUTO_OPERATE)
	{
		Auto_Move();
	}
    else if (Gimbal.ControlMode == GM_AIMBOT_OPERATE)
	{
		Auto_Move();
    }
    else if (Gimbal.ControlMode == GM_RESET_POSITION)
	{
        Gimbal.Command.Yaw = Gimbal.Imu.YawAngle;
        Gimbal.Command.Pitch = Gimbal.Imu.PitchAngle;
        fp32 YawTempCommand = loop_fp32_constrain(Gimbal.Imu.YawAngle, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
        Gimbal.Output.Yaw = YAW_MOTOR_DIRECTION * cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle, Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
        Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);
    }
    else
	{
        Gimbal.Command.Yaw = Gimbal.Imu.YawAngle;
        Gimbal.Command.Pitch = Gimbal.Imu.PitchAngle;
        Gimbal.Output.Yaw = 0;
        Gimbal.Output.Pitch = 0;
    }
}


void RotorCommandUpdate(void)
{
    if (Gimbal.FireMode == GM_FIRE_BUSY)
		{
        Gimbal.Command.Rotor = ROTOR_SPEEDSET_FORWARD * ROTOR_MOTOR_DIRECTION;
    }
    else if (Gimbal.FireMode == GM_FIRE_LAGGING)
		{
        Gimbal.Command.Rotor = ROTOR_SPEEDSET_BACKWARD * (-ROTOR_MOTOR_DIRECTION);
    }
    else if (Gimbal.FireMode == GM_FIRE_UNABLE)
		{
        Gimbal.Command.Rotor = 0;
        Gimbal.Output.Rotor = 0;
        return;
    }
    else{
        Gimbal.Command.Rotor = 0;
    }
    
    Gimbal.Output.Rotor = PID_calc(&Gimbal.Pid.Rotor, Gimbal.MotorMeasure.ShootMotor.RotorMotorSpeed, Gimbal.Command.Rotor);
}
/*******************************SetSpeed屏幕回传值******************************************/
void AmmoCommandUpdate(void)
{
    if (Gimbal.FireMode == GM_FIRE_UNABLE)
		{
        Gimbal.Command.AmmoLeft = 0;
        Gimbal.Command.AmmoRight = 0;
        Gimbal.Output.AmmoLeft = 0;
        Gimbal.Output.AmmoRight = 0;
        return;
    }
	SetSpeed = AMMO_SPEEDSET_30MS;
    switch (MSthis){
        case 30:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                SetSpeed * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                SetSpeed * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
        
        default:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                SetSpeed * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                SetSpeed * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
    }
    
}

void GetGimbalMotorOutput(GimbalOutput_t *out)
{
    memcpy(out, &Gimbal.Output, sizeof(GimbalOutput_t));
}

void DebugLEDShow(void)
{
    if ((Offline.AimbotStateNode == DEVICE_ONLINE)  &&  (Offline.AimbotDataNode == DEVICE_ONLINE))
	{
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        if ((Aimbot.State & AIMBOT_TARGET_INSIDE_OFFSET))
		{
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
        }
        else
		{
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
        }
    }
    else
	{
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
    }
    
    
}
fp32 LimitNormalization(fp32 input)
{
    if (input > 1.0f){
        return 1.0f;
    }
    else if (input < -1.0f){
        return -1.0f;
    }
    else{
        return input;
    }
}

void RefereeHeat_init(void)
{
	Gimbal.Referee.CoolingHeat = 100;
	Gimbal.Referee.MaxHeat = 320;

}

void RefereeHeatInterpolation(void)
{
    Gimbal.Referee.CurrentHeat -= Gimbal.Referee.CoolingHeat / 10;
    if (Gimbal.Referee.CurrentHeat < 0) 
	{
        Gimbal.Referee.CurrentHeat = 0;
    }
}




