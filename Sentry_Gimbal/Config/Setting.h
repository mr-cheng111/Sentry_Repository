#ifndef SETTING_H
#define SETTING_H

#define MOTOR_OFFLINE_TIMEMAX   50
#define REMOTE_OFFLINE_TIMEMAX  550
#define AIMBOT_OFFLINE_TIMEMAX  550
#define REFEREE_OFFLINE_TIMEMAX 3000

#define YAW_REMOTE_SENS                         0.25f
#define PITCH_REMOTE_SENS                       0.25f
#define YAW_MOUSE_SENS                          100
#define PITCH_MOUSE_SENS                        50


#define SENTRY7_UP_GIMBAL 

#ifdef  SENTRY7_UP_GIMBAL
// ���������ļ�
#define PARAMETER_FILE "Sentry7Parameter.h"
// imu��װ����
#define IMU_DIRECTION_yrxz_XYZ
// ���ID����
#define YAW_MOTOR_ID                0x205
#define PITCH_MOTOR_ID              0x207
#define ROTOR_MOTOR_ID              0x203
#define AMMO_LEFT_MOTOR_ID          0x201
#define AMMO_RIGHT_MOTOR_ID         0x202
// �����װ����
// ��̨��������˶��������̨��̬�����ƣ�����ϵͬ��Ϊ1������Ϊ-1
// ���̵�������˶�����͵������ǹ�ܷ���ͬ��Ϊ1������Ϊ-1
// Ħ���ֵ�������˶�����͵���ͬ��Ϊ1������Ϊ-1
#define YAW_MOTOR_DIRECTION         1
#define PITCH_MOTOR_DIRECTION       1
#define ROTOR_MOTOR_DIRECTION       -1
#define AMMO_LEFT_MOTOR_DIRECTION   -1
#define AMMO_RIGHT_MOTOR_DIRECTION  1
// ��̨YAW�����͸����޷�
#define YAW_ZERO_ECDANGLE           -44.0f
#define PITCH_MIN_ANGLE             -8.0f
#define PITCH_MAX_ANGLE             44.0f
// Ĭ��Ħ�����ٶ�
#define DEFAULT_AMMOL_PID           AMMO_LEFT_SPEED_30MS
#define DEFAULT_AMMOR_PID           AMMO_RIGHT_SPEED_30MS
#define DEFAULT_AMMO_SPEEDSET       AMMO_SPEEDSET_30MS
// ͨ��can����λ��
#define COMMUNICATE_CANPORT         hcan1

#else
#define PARAMETER_FILE "Sentry7Parameter.h"

//imu��װ����
#define IMU_DIRECTION_ryxz_XYZ
//���ID����
#define YAW_MOTOR_ID                0x205
#define PITCH_MOTOR_ID              0x206
#define ROTOR_MOTOR_ID              0x203
#define AMMO_LEFT_MOTOR_ID          0x201
#define AMMO_RIGHT_MOTOR_ID         0x202
//�����װ����
//��̨��������˶��������̨��̬�����ƣ�����ϵͬ��Ϊ1������Ϊ-1
//���̵�������˶�����͵������ǹ�ܷ���ͬ��Ϊ1������Ϊ-1
//Ħ���ֵ�������˶�����͵���ͬ��Ϊ1������Ϊ-1
#define YAW_MOTOR_DIRECTION         -1
#define PITCH_MOTOR_DIRECTION       -1
#define ROTOR_MOTOR_DIRECTION       1
#define AMMO_LEFT_MOTOR_DIRECTION   -1
#define AMMO_RIGHT_MOTOR_DIRECTION  1
// ��̨YAW�����͸����޷�
#define YAW_ZERO_ECDANGLE           0.0f
#define PITCH_MIN_ANGLE             -12.8f
#define PITCH_MAX_ANGLE             40.0f
// Ĭ��Ħ�����ٶ�
#define DEFAULT_AMMOL_PID           AMMO_LEFT_SPEED_30MS
#define DEFAULT_AMMOR_PID           AMMO_RIGHT_SPEED_30MS
#define DEFAULT_AMMO_SPEEDSET       AMMO_SPEEDSET_30MS
// ͨ��can����λ��
#define COMMUNICATE_CANPORT         hcan1
#endif

#endif
//#define IMU_DIRECTION_xyz_XYZ
//#define IMU_DIRECTION_yrxz_XYZ
//#define IMU_DIRECTION_rxryz_XYZ
//#define IMU_DIRECTION_ryxz_XYZ

//#define IMU_DIRECTION_zryx_XYZ
//#define IMU_DIRECTION_yzx_XYZ
//#define IMU_DIRECTION_rzyx_XYZ
//#define IMU_DIRECTION_ryrzx_XYZ
