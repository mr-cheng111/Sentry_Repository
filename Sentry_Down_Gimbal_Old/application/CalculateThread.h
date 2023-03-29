#ifndef __CalculateThread_H
#define __CalculateThread_H

#include "struct_typedef.h"
#include "CanPacket.h"

#define TEST_MODE  1 
#define GAME_MODE  2

typedef struct{
    fp32    Yaw;
    fp32    Pitch;
    fp32    Rotor;
    fp32    AmmoLeft;
    fp32    AmmoRight;
} GimbalCommand_t;

typedef struct{
    int16_t    Yaw;
    int16_t    Pitch;
    int16_t    Rotor;
    int16_t    AmmoLeft;
    int16_t    AmmoRight;
} GimbalOutput_t;

typedef struct{
    cascade_pid_t   Yaw;
    cascade_pid_t   Pitch;
    pid_type_def    Rotor;
    pid_type_def    AmmoLeft;
    pid_type_def    AmmoRight;
} GimbalPID_t;

typedef enum{
    GM_NO_FORCE         =   0x00,
    GM_INIT                     ,
    GM_TEST                     ,
    GM_MATCH                    ,
} GimbalStateMachine_e;

typedef enum{
    GM_NO_CONTROL       =   0x00,
    GM_RESET_POSITION           ,
    GM_MANUAL_OPERATE           ,
    GM_AIMBOT_OPERATE           ,
    GM_AIMBOT_RUNES             ,
	GM_AUTO_OPERATE				,
} GimbalControlMode_e;

typedef enum{
    GM_FIRE_UNABLE      =   0x00,
    GM_FIRE_READY               ,
    GM_FIRE_BUSY                ,
    GM_FIRE_COOLING             ,
    GM_FIRE_LAGGING             ,
} GimbalFireMode_e;

typedef struct{
    ShootMotorMeasure_t     ShootMotor;
    GimbalMotorMeasure_t    GimbalMotor;
} MotorMeasure_t;

typedef struct{
    uint8_t     Camp;
    uint8_t     MaxSpeed;
    fp32        CurrentSpeed;
    uint16_t    MaxHeat;
    uint16_t    CoolingHeat;
    int16_t     CurrentHeat;
} RefereeMeasure_t;

typedef struct{
    fp32                    YawAddress[64];
    fp32                    PitchAddress[64];
    LoopFifoFp32_t          YawLoopPointer;
    LoopFifoFp32_t          PitchLoopPointer;
} ImuBuffer_t;

typedef struct{
    EulerSystemMeasure_t    Imu;                    //  imu�����ݷ���
    MotorMeasure_t          MotorMeasure;           //  ��������������ݷ���
    GimbalCommand_t         Command;                //  ��̨�Ƕȿ���ָ��
    GimbalPID_t             Pid;                    //  ��̨PID�ṹ��
    GimbalOutput_t          Output;                 //  ��̨������
    GimbalStateMachine_e    StateMachine;           //  ��̨״̬��
    GimbalControlMode_e     ControlMode;            //  ��̨����ģʽ
    GimbalFireMode_e        FireMode;               //  ��̨����ģʽ
    RefereeMeasure_t        Referee;                //  ����ϵͳ����
    ImuBuffer_t             ImuBuffer;
	uint8_t					Chassis_Mode;
} Gimbal_t;

typedef enum
{
	CHASSIS_WEEK = 0,
	CHASSIS_STOP,
	CHASSIS_FAST,
	CHASSIS_BRAKE

}Chassis_Mode_Type;

extern Gimbal_t              Gimbal;
extern uint32_t    			 Work_Mode;


extern void CalculateThread(void const * pvParameters);

extern void GetGimbalMotorOutput(GimbalOutput_t *out);
extern void GetGimbalRequestState(GimbalRequestState_t *RequestState);

extern void RefereeHeatInterpolation(void);

#endif



