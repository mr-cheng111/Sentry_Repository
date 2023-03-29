#ifndef SETTING_H
#define SETTING_H

#define MOTOR_OFFLINE_TIMEMAX   50
#define REMOTE_OFFLINE_TIMEMAX  550
#define AIMBOT_OFFLINE_TIMEMAX  550
#define REFEREE_OFFLINE_TIMEMAX 3000

#define SENTRY7
// 参数配置文件
#define PARAMETER_FILE "Sentry7Parameter.h"
// 键位配置文件
#define KEYMAP_FILE "Sentry7KeyMap.h"

//imu安装方向
#define IMU_DIRECTION_ryxz_XYZ
//电机ID分配
#define YAW_MOTOR_ID                0x205
#define PITCH_MOTOR_ID              0x206
#define ROTOR_MOTOR_ID              0x203
#define AMMO_LEFT_MOTOR_ID          0x201
#define AMMO_RIGHT_MOTOR_ID         0x202
//电机安装方向
//云台电机正向运动方向和云台姿态（控制）坐标系同向为1，反向为-1
//拨盘电机正向运动方向和弹丸进入枪管方向同向为1，反向为-1
//摩擦轮电机正向运动方向和弹道同向为1，反向为-1
#define YAW_MOTOR_DIRECTION         -1
#define PITCH_MOTOR_DIRECTION       -1
#define ROTOR_MOTOR_DIRECTION       1
#define AMMO_LEFT_MOTOR_DIRECTION   -1
#define AMMO_RIGHT_MOTOR_DIRECTION  1
// 云台YAW轴零点和俯仰限幅
#define YAW_ZERO_ECDANGLE           0.0f
#define PITCH_MIN_ANGLE             -39.0f
#define PITCH_MAX_ANGLE             21.0f
// 默认摩擦轮速度
#define DEFAULT_AMMOL_PID           AMMO_LEFT_SPEED_30MS
#define DEFAULT_AMMOR_PID           AMMO_RIGHT_SPEED_30MS
#define DEFAULT_AMMO_SPEEDSET       AMMO_SPEEDSET_30MS
// 通信can总线位置
#define COMMUNICATE_CANPORT         hcan1

//#define IMU_DIRECTION_xyz_XYZ
//#define IMU_DIRECTION_yrxz_XYZ
//#define IMU_DIRECTION_rxryz_XYZ
//#define IMU_DIRECTION_ryxz_XYZ

//#define IMU_DIRECTION_zryx_XYZ
//#define IMU_DIRECTION_yzx_XYZ
//#define IMU_DIRECTION_rzyx_XYZ
//#define IMU_DIRECTION_ryrzx_XYZ




#endif

