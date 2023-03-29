#ifndef __SENTRY7_PARAMETER_H
#define __SENTRY7_PARAMETER_H

#include "struct_typedef.h"
#include "Setting.h"


#ifdef  SENTRY7_UP_GIMBAL
/************************************************************************上云台参数*****************************************************************************************/
#define GM6020_MAX_OUTPUT                       30000
#define GM6020_MAX_IOUTPUT                      10000
#define M3508_MAX_OUTPUT                        16384
#define M3508_MAX_IOUTPUT                       6000
#define M2006_MAX_OUTPUT                        10000
#define M2006_MAX_IOUTPUT                       5000

#define PITCH_MAX_SPEED                         300
#define PITCH_MAX_ISPEED                        30
#define YAW_MAX_SPEED                           300
#define YAW_MAX_ISPEED                          30
//射速
#define AMMO_SPEEDSET_30MS                      7300

#define ROTOR_SPEEDSET_FORWARD                  3000.0f
#define ROTOR_SPEEDSET_BACKWARD                 1000.0f 

//扫射拨盘前转时间 
#define BUSTFIRE_ROTOR_TIMESET_BUSY             60
//扫射拨盘旋转时间间隔
#define BUSTFIRE_ROTOR_TIMESET_COOLING          10

//点射拨盘前转时间 
#define POINTFIRE_ROTOR_TIMESET_BUSY_ONE        40
//点射拨盘旋转时间间隔
#define POINTFIRE_ROTOR_TIMESET_COOLING_ONE     50

//点射拨盘前转时间 
#define POINTFIRE_ROTOR_TIMESET_BUSY_TWO        25
//点射拨盘旋转时间间隔
#define POINTFIRE_ROTOR_TIMESET_COOLING_TWO     140
//拨盘反转时间
#define ROTOR_TIMESET_RESERVE                   80
//拨盘反转检测时间
#define ROTOR_LAGGING_COUNTER_MAX               70

#define MAX_HEAT 								320
//  无力云台参数
//  YAW轴角速度环
#define YAW_SPEED_NO_FORCE_KP                   0.0f
#define YAW_SPEED_NO_FORCE_KI                   0.0f
#define YAW_SPEED_NO_FORCE_KD                   0.0f
fp32 YAW_SPEED_NO_FORCE[3] = {YAW_SPEED_NO_FORCE_KP, YAW_SPEED_NO_FORCE_KI, YAW_SPEED_NO_FORCE_KD};
//  YAW轴角度环
#define YAW_ANGLE_NO_FORCE_KP                   0.0f
#define YAW_ANGLE_NO_FORCE_KI                   0.0f
#define YAW_ANGLE_NO_FORCE_KD                   0.0f
fp32 YAW_ANGLE_NO_FORCE[3] = {YAW_ANGLE_NO_FORCE_KP, YAW_ANGLE_NO_FORCE_KI, YAW_ANGLE_NO_FORCE_KD};
//  PITCH轴角速度环
#define PITCH_SPEED_NO_FORCE_KP                 0.0f
#define PITCH_SPEED_NO_FORCE_KI                 0.0f
#define PITCH_SPEED_NO_FORCE_KD                 0.0f
fp32 PITCH_SPEED_NO_FORCE[3] = {PITCH_SPEED_NO_FORCE_KP, PITCH_SPEED_NO_FORCE_KI, PITCH_SPEED_NO_FORCE_KD};
//  PITCH轴角度环
#define PITCH_ANGLE_NO_FORCE_KP                 0.0f
#define PITCH_ANGLE_NO_FORCE_KI                 0.0f
#define PITCH_ANGLE_NO_FORCE_KD                 0.0f
fp32 PITCH_ANGLE_NO_FORCE[3] = {PITCH_ANGLE_NO_FORCE_KP, PITCH_ANGLE_NO_FORCE_KI, PITCH_ANGLE_NO_FORCE_KD};

//  手动控制云台参数
//  YAW轴角速度环
#define YAW_SPEED_MANUAL_OPERATE_KP             180.0f
#define YAW_SPEED_MANUAL_OPERATE_KI             0.0f
#define YAW_SPEED_MANUAL_OPERATE_KD             0.0f
fp32 YAW_SPEED_MANUAL_OPERATE[3] = {YAW_SPEED_MANUAL_OPERATE_KP, YAW_SPEED_MANUAL_OPERATE_KI, YAW_SPEED_MANUAL_OPERATE_KD};
//  YAW轴角度环
#define YAW_ANGLE_MANUAL_OPERATE_KP             60.0f
#define YAW_ANGLE_MANUAL_OPERATE_KI             0.0f
#define YAW_ANGLE_MANUAL_OPERATE_KD             0.0f
fp32 YAW_ANGLE_MANUAL_OPERATE[3] = {YAW_ANGLE_MANUAL_OPERATE_KP, YAW_ANGLE_MANUAL_OPERATE_KI, YAW_ANGLE_MANUAL_OPERATE_KD};
//  PITCH轴角速度环
#define PITCH_SPEED_MANUAL_OPERATE_KP           160.0f
#define PITCH_SPEED_MANUAL_OPERATE_KI           0.0f
#define PITCH_SPEED_MANUAL_OPERATE_KD           0.0f
fp32 PITCH_SPEED_MANUAL_OPERATE[3] = {PITCH_SPEED_MANUAL_OPERATE_KP, PITCH_SPEED_MANUAL_OPERATE_KI, PITCH_SPEED_MANUAL_OPERATE_KD};
//  PITCH轴角度环
#define PITCH_ANGLE_MANUAL_OPERATE_KP           45.0f
#define PITCH_ANGLE_MANUAL_OPERATE_KI           0.0f
#define PITCH_ANGLE_MANUAL_OPERATE_KD           0.0f
fp32 PITCH_ANGLE_MANUAL_OPERATE[3] = {PITCH_ANGLE_MANUAL_OPERATE_KP, PITCH_ANGLE_MANUAL_OPERATE_KI, PITCH_ANGLE_MANUAL_OPERATE_KD};
//  自瞄控制云台参数
//  YAW轴角速度环
#define YAW_SPEED_AIMBOT_OPERATE_KP             160.0f
#define YAW_SPEED_AIMBOT_OPERATE_KI             0.0f
#define YAW_SPEED_AIMBOT_OPERATE_KD             0.0f
fp32 YAW_SPEED_AIMBOT_OPERATE[3] = {YAW_SPEED_AIMBOT_OPERATE_KP, YAW_SPEED_AIMBOT_OPERATE_KI, YAW_SPEED_AIMBOT_OPERATE_KD};
//  YAW轴角度环
#define YAW_ANGLE_AIMBOT_OPERATE_KP             60.0f
#define YAW_ANGLE_AIMBOT_OPERATE_KI             0.10f
#define YAW_ANGLE_AIMBOT_OPERATE_KD             0.0f
fp32 YAW_ANGLE_AIMBOT_OPERATE[3] = {YAW_ANGLE_AIMBOT_OPERATE_KP, YAW_ANGLE_AIMBOT_OPERATE_KI, YAW_ANGLE_AIMBOT_OPERATE_KD};
//  PITCH轴角速度环
#define PITCH_SPEED_AIMBOT_OPERATE_KP           180.0f
#define PITCH_SPEED_AIMBOT_OPERATE_KI           0.0f
#define PITCH_SPEED_AIMBOT_OPERATE_KD           0.0f
fp32 PITCH_SPEED_AIMBOT_OPERATE[3] = {PITCH_SPEED_AIMBOT_OPERATE_KP, PITCH_SPEED_AIMBOT_OPERATE_KI, PITCH_SPEED_AIMBOT_OPERATE_KD};
//  PITCH轴角度环
#define PITCH_ANGLE_AIMBOT_OPERATE_KP           45.0f
#define PITCH_ANGLE_AIMBOT_OPERATE_KI           0.0f
#define PITCH_ANGLE_AIMBOT_OPERATE_KD           0.0f
fp32 PITCH_ANGLE_AIMBOT_OPERATE[3] = {PITCH_ANGLE_AIMBOT_OPERATE_KP, PITCH_ANGLE_AIMBOT_OPERATE_KI, PITCH_ANGLE_AIMBOT_OPERATE_KD};

//  摩擦轮参数
//点射
#define AMMO_LEFT_SPEED_30MS_KP                 25.0f
#define AMMO_LEFT_SPEED_30MS_KI                 0.0f
#define AMMO_LEFT_SPEED_30MS_KD                 0.0f
fp32 AMMO_LEFT_SPEED_30MS[3] = {AMMO_LEFT_SPEED_30MS_KP, AMMO_LEFT_SPEED_30MS_KI, AMMO_LEFT_SPEED_30MS_KD};

#define AMMO_RIGHT_SPEED_30MS_KP                25.0f
#define AMMO_RIGHT_SPEED_30MS_KI                0.0f
#define AMMO_RIGHT_SPEED_30MS_KD                0.0f
fp32 AMMO_RIGHT_SPEED_30MS[3] = {AMMO_RIGHT_SPEED_30MS_KP, AMMO_RIGHT_SPEED_30MS_KI, AMMO_RIGHT_SPEED_30MS_KD};

//扫射
#define AMMO_BURST_LEFT_SPEED_30MS_KP                 25.0f
#define AMMO_BURST_LEFT_SPEED_30MS_KI                 0.0f
#define AMMO_BURST_LEFT_SPEED_30MS_KD                 0.0f
fp32 	AMMO_BURST_LEFT_SPEED_30MS[3] = {AMMO_LEFT_SPEED_30MS_KP, AMMO_LEFT_SPEED_30MS_KI, AMMO_LEFT_SPEED_30MS_KD};

#define AMMO_BURST_RIGHT_SPEED_30MS_KP                25.0f
#define AMMO_BURST_RIGHT_SPEED_30MS_KI                0.0f
#define AMMO_BURST_RIGHT_SPEED_30MS_KD                0.0f
fp32 	AMMO_BURST_RIGHT_SPEED_30MS[3] = {AMMO_RIGHT_SPEED_30MS_KP, AMMO_RIGHT_SPEED_30MS_KI, AMMO_RIGHT_SPEED_30MS_KD};

#define ROTOR_UNABLE_KP                         0.0f
#define ROTOR_UNABLE_KI                         0.0f
#define ROTOR_UNABLE_KD                         0.0f
fp32 ROTOR_UNABLE[3] = {ROTOR_UNABLE_KP, ROTOR_UNABLE_KI, ROTOR_UNABLE_KD};

#define ROTOR_FORWARD_KP                        120.0f
#define ROTOR_FORWARD_KI                        0.0f
#define ROTOR_FORWARD_KD                        0.0f
fp32 ROTOR_FORWARD[3] = {ROTOR_FORWARD_KP, ROTOR_FORWARD_KI, ROTOR_FORWARD_KD};

#define ROTOR_STOP_KP                           30.0f
#define ROTOR_STOP_KI                           0.0f
#define ROTOR_STOP_KD                           0.0f
fp32 ROTOR_STOP[3] = {ROTOR_STOP_KP, ROTOR_STOP_KI, ROTOR_STOP_KD};

#define ROTOR_BACK_KP                           60.0f
#define ROTOR_BACK_KI                           0.0f
#define ROTOR_BACK_KD                           0.0f
fp32 ROTOR_BACK[3] = {ROTOR_BACK_KP, ROTOR_BACK_KI, ROTOR_BACK_KD};

#else



/************************************************************************下云台参数*****************************************************************************************/

//yaw、pitch轴最大电流输出
#define GM6020_MAX_OUTPUT                       30000
#define GM6020_MAX_IOUTPUT                      10000
#define M3508_MAX_OUTPUT                        16384
#define M3508_MAX_IOUTPUT                       6000
#define M2006_MAX_OUTPUT                        10000
#define M2006_MAX_IOUTPUT                       5000


//yaw、pitch轴最大速度输出
#define PITCH_MAX_SPEED                         300
#define PITCH_MAX_ISPEED                        30
#define YAW_MAX_SPEED                           300
#define YAW_MAX_ISPEED                          30

//射速
#define AMMO_SPEEDSET_30MS                      7600

//拨盘前转速度
#define ROTOR_SPEEDSET_FORWARD                  3000.0f
//拨盘反转速度
#define ROTOR_SPEEDSET_BACKWARD                 1000.0f

//扫射拨盘前转时间 
#define BUSTFIRE_ROTOR_TIMESET_BUSY             45
//扫射拨盘旋转时间间隔
#define BUSTFIRE_ROTOR_TIMESET_COOLING          1

//点射拨盘前转时间 
#define POINTFIRE_ROTOR_TIMESET_BUSY_ONE        30
//点射拨盘旋转时间间隔
#define POINTFIRE_ROTOR_TIMESET_COOLING_ONE     50

//点射拨盘前转时间 
#define POINTFIRE_ROTOR_TIMESET_BUSY_TWO        25
//点射拨盘旋转时间间隔
#define POINTFIRE_ROTOR_TIMESET_COOLING_TWO     130


//拨盘反转时间
#define ROTOR_TIMESET_RESERVE                   80
//拨盘反转检测时间
#define ROTOR_LAGGING_COUNTER_MAX               70

#define MAX_HEAT 								320

//  无力云台参数
//  YAW轴角速度环
#define YAW_SPEED_NO_FORCE_KP                   0.0f
#define YAW_SPEED_NO_FORCE_KI                   0.0f
#define YAW_SPEED_NO_FORCE_KD                   0.0f
fp32 YAW_SPEED_NO_FORCE[3] = {YAW_SPEED_NO_FORCE_KP, YAW_SPEED_NO_FORCE_KI, YAW_SPEED_NO_FORCE_KD};
//  YAW轴角度环
#define YAW_ANGLE_NO_FORCE_KP                   0.0f
#define YAW_ANGLE_NO_FORCE_KI                   0.0f
#define YAW_ANGLE_NO_FORCE_KD                   0.0f
fp32 YAW_ANGLE_NO_FORCE[3] = {YAW_ANGLE_NO_FORCE_KP, YAW_ANGLE_NO_FORCE_KI, YAW_ANGLE_NO_FORCE_KD};
//  PITCH轴角速度环
#define PITCH_SPEED_NO_FORCE_KP                 0.0f
#define PITCH_SPEED_NO_FORCE_KI                 0.0f
#define PITCH_SPEED_NO_FORCE_KD                 0.0f
fp32 PITCH_SPEED_NO_FORCE[3] = {PITCH_SPEED_NO_FORCE_KP, PITCH_SPEED_NO_FORCE_KI, PITCH_SPEED_NO_FORCE_KD};
//  PITCH轴角度环
#define PITCH_ANGLE_NO_FORCE_KP                 0.0f
#define PITCH_ANGLE_NO_FORCE_KI                 0.0f
#define PITCH_ANGLE_NO_FORCE_KD                 0.0f
fp32 PITCH_ANGLE_NO_FORCE[3] = {PITCH_ANGLE_NO_FORCE_KP, PITCH_ANGLE_NO_FORCE_KI, PITCH_ANGLE_NO_FORCE_KD};


//  归位云台参数
//  YAW轴角速度环
#define YAW_SPEED_RESET_POSITION_KP             36.0f
#define YAW_SPEED_RESET_POSITION_KI             0.0f
#define YAW_SPEED_RESET_POSITION_KD             0.0f
fp32 YAW_SPEED_RESET_POSITION[3] = {YAW_SPEED_RESET_POSITION_KP, YAW_SPEED_RESET_POSITION_KI, YAW_SPEED_RESET_POSITION_KD};
//  YAW轴角度环
#define YAW_ANGLE_RESET_POSITION_KP             18.0f
#define YAW_ANGLE_RESET_POSITION_KI             0.0f
#define YAW_ANGLE_RESET_POSITION_KD             0.0f
fp32 YAW_ANGLE_RESET_POSITION[3] = {YAW_ANGLE_RESET_POSITION_KP, YAW_ANGLE_RESET_POSITION_KI, YAW_ANGLE_RESET_POSITION_KD};
//  PITCH轴角速度环
#define PITCH_SPEED_RESET_POSITION_KP           160.0f
#define PITCH_SPEED_RESET_POSITION_KI           0.0f
#define PITCH_SPEED_RESET_POSITION_KD           0.0f
fp32 PITCH_SPEED_RESET_POSITION[3] = {PITCH_SPEED_RESET_POSITION_KP, PITCH_SPEED_RESET_POSITION_KI, PITCH_SPEED_RESET_POSITION_KD};
//  PITCH轴角度环
#define PITCH_ANGLE_RESET_POSITION_KP           60.0f
#define PITCH_ANGLE_RESET_POSITION_KI           0.0f
#define PITCH_ANGLE_RESET_POSITION_KD           0.0f
fp32 PITCH_ANGLE_RESET_POSITION[3] = {PITCH_ANGLE_RESET_POSITION_KP, PITCH_ANGLE_RESET_POSITION_KI, PITCH_ANGLE_RESET_POSITION_KD};


//  手动控制云台参数
//  YAW轴角速度环
#define YAW_SPEED_MANUAL_OPERATE_KP             180.0f
#define YAW_SPEED_MANUAL_OPERATE_KI             0.0f
#define YAW_SPEED_MANUAL_OPERATE_KD             0.0f
fp32 YAW_SPEED_MANUAL_OPERATE[3] = {YAW_SPEED_MANUAL_OPERATE_KP, YAW_SPEED_MANUAL_OPERATE_KI, YAW_SPEED_MANUAL_OPERATE_KD};
//  YAW轴角度环
#define YAW_ANGLE_MANUAL_OPERATE_KP             25.0f
#define YAW_ANGLE_MANUAL_OPERATE_KI             0.0f
#define YAW_ANGLE_MANUAL_OPERATE_KD             0.0f
fp32 YAW_ANGLE_MANUAL_OPERATE[3] = {YAW_ANGLE_MANUAL_OPERATE_KP, YAW_ANGLE_MANUAL_OPERATE_KI, YAW_ANGLE_MANUAL_OPERATE_KD};
//  PITCH轴角速度环
#define PITCH_SPEED_MANUAL_OPERATE_KP           160.0f
#define PITCH_SPEED_MANUAL_OPERATE_KI           0.0f
#define PITCH_SPEED_MANUAL_OPERATE_KD           0.0f
fp32 PITCH_SPEED_MANUAL_OPERATE[3] = {PITCH_SPEED_MANUAL_OPERATE_KP, PITCH_SPEED_MANUAL_OPERATE_KI, PITCH_SPEED_MANUAL_OPERATE_KD};
//  PITCH轴角度环
#define PITCH_ANGLE_MANUAL_OPERATE_KP           60.0f
#define PITCH_ANGLE_MANUAL_OPERATE_KI           0.0f
#define PITCH_ANGLE_MANUAL_OPERATE_KD           0.0f
fp32 PITCH_ANGLE_MANUAL_OPERATE[3] = {PITCH_ANGLE_MANUAL_OPERATE_KP, PITCH_ANGLE_MANUAL_OPERATE_KI, PITCH_ANGLE_MANUAL_OPERATE_KD};



//  自瞄控制云台参数

//  自瞄控制云台参数
//  YAW轴角速度环
#define YAW_SPEED_AIMBOT_OPERATE_KP             170.0f
#define YAW_SPEED_AIMBOT_OPERATE_KI             0.0f
#define YAW_SPEED_AIMBOT_OPERATE_KD             0.0f
fp32 YAW_SPEED_AIMBOT_OPERATE[3] = {YAW_SPEED_AIMBOT_OPERATE_KP, YAW_SPEED_AIMBOT_OPERATE_KI, YAW_SPEED_AIMBOT_OPERATE_KD};
//  YAW轴角度环
#define YAW_ANGLE_AIMBOT_OPERATE_KP             25.0f
#define YAW_ANGLE_AIMBOT_OPERATE_KI             0.1f
#define YAW_ANGLE_AIMBOT_OPERATE_KD             0.0f
fp32 YAW_ANGLE_AIMBOT_OPERATE[3] = {YAW_ANGLE_AIMBOT_OPERATE_KP, YAW_ANGLE_AIMBOT_OPERATE_KI, YAW_ANGLE_AIMBOT_OPERATE_KD};
//  PITCH轴角速度环
#define PITCH_SPEED_AIMBOT_OPERATE_KP           160.0f
#define PITCH_SPEED_AIMBOT_OPERATE_KI           0.0f
#define PITCH_SPEED_AIMBOT_OPERATE_KD           0.0f
fp32 PITCH_SPEED_AIMBOT_OPERATE[3] = {PITCH_SPEED_AIMBOT_OPERATE_KP, PITCH_SPEED_AIMBOT_OPERATE_KI, PITCH_SPEED_AIMBOT_OPERATE_KD};
//  PITCH轴角度环
#define PITCH_ANGLE_AIMBOT_OPERATE_KP           30.0f
#define PITCH_ANGLE_AIMBOT_OPERATE_KI           0.01f
#define PITCH_ANGLE_AIMBOT_OPERATE_KD           0.0f
fp32 PITCH_ANGLE_AIMBOT_OPERATE[3] = {PITCH_ANGLE_AIMBOT_OPERATE_KP, PITCH_ANGLE_AIMBOT_OPERATE_KI, PITCH_ANGLE_AIMBOT_OPERATE_KD};

//  自动模式下云台参数
//  YAW轴角速度环
#define YAW_SPEED_AUTO_POSITION_KP             36.0f
#define YAW_SPEED_AUTO_POSITION_KI             0.0f
#define YAW_SPEED_AUTO_POSITION_KD             0.0f
fp32 YAW_SPEED_AUTO_POSITION[3] = {YAW_SPEED_AUTO_POSITION_KP, YAW_SPEED_AUTO_POSITION_KI, YAW_SPEED_AUTO_POSITION_KD};
//  YAW轴角度环
#define YAW_ANGLE_AUTO_POSITION_KP             18.0f
#define YAW_ANGLE_AUTO_POSITION_KI             0.0f
#define YAW_ANGLE_AUTO_POSITION_KD             0.0f
fp32 YAW_ANGLE_AUTO_POSITION[3] = {YAW_ANGLE_AUTO_POSITION_KP, YAW_ANGLE_AUTO_POSITION_KI, YAW_ANGLE_AUTO_POSITION_KD};
//  PITCH轴角速度环
#define PITCH_SPEED_AUTO_POSITION_KP           160.0f
#define PITCH_SPEED_AUTO_POSITION_KI           0.0f
#define PITCH_SPEED_AUTO_POSITION_KD           0.0f
fp32 PITCH_SPEED_AUTO_POSITION[3] = {PITCH_SPEED_AUTO_POSITION_KP, PITCH_SPEED_AUTO_POSITION_KI, PITCH_SPEED_AUTO_POSITION_KD};
//  PITCH轴角度环
#define PITCH_ANGLE_AUTO_POSITION_KP           60.0f
#define PITCH_ANGLE_AUTO_POSITION_KI           0.0f
#define PITCH_ANGLE_AUTO_POSITION_KD           0.0f
fp32 PITCH_ANGLE_AUTO_POSITION[3] = {PITCH_ANGLE_AUTO_POSITION_KP, PITCH_ANGLE_AUTO_POSITION_KI, PITCH_ANGLE_AUTO_POSITION_KD};

//  摩擦轮参数
#define AMMO_LEFT_SPEED_30MS_KP                 18.0f
#define AMMO_LEFT_SPEED_30MS_KI                 0.0f
#define AMMO_LEFT_SPEED_30MS_KD                 0.0f
fp32 AMMO_LEFT_SPEED_30MS[3] = {AMMO_LEFT_SPEED_30MS_KP, AMMO_LEFT_SPEED_30MS_KI, AMMO_LEFT_SPEED_30MS_KD};

#define AMMO_RIGHT_SPEED_30MS_KP                18.0f
#define AMMO_RIGHT_SPEED_30MS_KI                0.0f
#define AMMO_RIGHT_SPEED_30MS_KD                0.0f
fp32 AMMO_RIGHT_SPEED_30MS[3] = {AMMO_RIGHT_SPEED_30MS_KP, AMMO_RIGHT_SPEED_30MS_KI, AMMO_RIGHT_SPEED_30MS_KD};

//拨盘参数
#define ROTOR_UNABLE_KP                         0.0f
#define ROTOR_UNABLE_KI                         0.0f
#define ROTOR_UNABLE_KD                         0.0f
fp32 ROTOR_UNABLE[3] = {ROTOR_UNABLE_KP, ROTOR_UNABLE_KI, ROTOR_UNABLE_KD};

#define ROTOR_FORWARD_KP                        170.0f
#define ROTOR_FORWARD_KI                        0.0f
#define ROTOR_FORWARD_KD                        0.0f
fp32 ROTOR_FORWARD[3] = {ROTOR_FORWARD_KP, ROTOR_FORWARD_KI, ROTOR_FORWARD_KD};

#define ROTOR_STOP_KP                           30.0f
#define ROTOR_STOP_KI                           0.0f
#define ROTOR_STOP_KD                           0.0f
fp32 ROTOR_STOP[3] = {ROTOR_STOP_KP, ROTOR_STOP_KI, ROTOR_STOP_KD};

#define ROTOR_BACK_KP                           60.0f
#define ROTOR_BACK_KI                           0.0f
#define ROTOR_BACK_KD                           0.0f
fp32 ROTOR_BACK[3] = {ROTOR_BACK_KP, ROTOR_BACK_KI, ROTOR_BACK_KD};

#endif



#endif






