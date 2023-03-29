#include "imu.h"
#include "remote_control.h"
#include "pid.h"
#include "bsp_can.h"

//最大PID输出和最大PID积分输出
#define MAX_PID_YAW_SPEED_OUT 27000.0f//最大电流，不能在增大
#define MAX_PID_YAW_SPEED_IOUT 2000.0

#define MAX_PID_YAW_POSITION_OUT 800.0f
#define MAX_PID_YAW_POSITION_IOUT 2.0f

#define MAX_PID_PITCH_SPEED_OUT 27000.0f//最大电流，不能在增大
#define MAX_PID_PITCH_SPEED_IOUT 2000.0f

#define MAX_PID_PITCH_POSITION_OUT 500.0f
#define MAX_PID_PITCH_POSITION_IOUT 0.0f

#define MAX_PID_SHOOT_OUT 16000.0f
#define MAX_PID_SHOOT_IOUT 2000.0f

#define MAX_PID_TRIGGE_ANGLE_OUT 100.0f
#define MAX_PID_TRIGGE_ANGLE_IOUT 20.0f

#define MAX_PID_TRIGGE_SPEED_OUT 10000.0f
#define MAX_PID_TRIGGE_SPEED_IOUT 2000.0f





//底盘电机pid参数
#define MAX_CHASSIS_OUT 16000.0f
#define MAX_CHASSIS_IOUT 2000.0f

#define MAX_VX_OUT 16000.0f
#define MAX_VX_IOUT 0.0f

#define MAX_VY_OUT 16000.0f
#define MAX_VY_IOUT 0.0f

#define MAX_WZ_OUT 16000.0f
#define MAX_WZ_IOUT 0.0f


//yaw轴 pid
#define YAW_SPEED_KP 397.0f
#define YAW_POSITION_KP 16.0f

#define YAW_SPEED_KI 5.0f
#define YAW_POSITION_KI 0.7f

#define YAW_SPEED_KD 0.0
#define YAW_POSITION_KD 0.0

//pitch轴 pid
#define PITCH_SPEED_KP 1210.0f
#define PITCH_POSITION_KP 10.5f

#define PITCH_SPEED_KI 0.6f
#define PITCH_POSITION_KI 0.27f

#define PITCH_SPEED_KD 0.0
#define PITCH_POSITION_KD 0.0


//拨盘 pid
#define TRIGGE_ANGLE_KP 1.0f
#define TRIGGE_ANGLE_KI 0.0f
#define TRIGGE_ANGLE_KD 0.0f

#define TRIGGE_SPEED_KP 1.0f
#define TRIGGE_SPEED_KI 0.0f
#define TRIGGE_SPEED_KD 0.0f

//地盘四个电机的pid
#define Chassis1_KP 30
#define Chassis2_KP 30
#define Chassis3_KP 30
#define Chassis4_KP 30

#define Chassis1_KI 0
#define Chassis2_KI 0
#define Chassis3_KI 0
#define Chassis4_KI 0

#define Chassis1_KD 0
#define Chassis2_KD 0
#define Chassis3_KD 0
#define Chassis4_KD 0


//摩擦轮pid
#define SHOOT1_KP 100
#define SHOOT2_KP 100

#define SHOOT1_KI 0
#define SHOOT2_KI 0

#define SHOOT1_KD 0
#define SHOOT2_KD 0



//底盘三个运动方向pid
#define VX_P 4;
#define VY_P 4;
#define WZ_P 4;

#define VX_I 0;
#define VY_I 0;
#define WZ_I 0;

#define VX_D 0;
#define VY_D 0;
#define WZ_D 0;





//控制云台灵敏度
#define YAW_SEN    -0.00045
#define PITCH_SEN -0.0001


//云台控制模式
#define SERVO 1
#define WEEK 3
#define ROTATING 2

#define SHOOT_START_FLAG 1
#define SHOOT_STOP_FLAG 3


#define MAX_PITCH_ANGLE 28
#define MIN_PITCH_ANGLE -38

#define MAX_YAW_ANGLE 3
#define MIN_YAW_ANGLE -3


//数学定义
#define PI 3.1415926


//控制角度方向 
#define YAW_ANGLE_DERECTION -1
#define PITCH_ANGLE_DERECTION 1
//控制电流方向
#define YAW_CURRENT_DERECTION 1
#define PITCH_CURRENT_DERECTION 1

//底盘电机旋转方向
#define MOTOR1 1
#define MOTOR2 1
#define MOTOR3 1
#define MOTOR4 1
#define MOTOR5 1

//定义6020机械角度精度
#define MACHINE_ANGLE 8191


#define SHOOT1_CURRENT_DERECTION 1
#define SHOOT2_CURRENT_DERECTION -1

#define SHOOT_SPEED 10000
#define SHOOT_STOP 0


#define VX_SPEED -15
#define VY_SPEED 15
#define WZ_SPEED 10

#define VX_SPEED_DERECTION 1
#define VY_SPEED_DERECTION 1
#define WZ_SPEED_DERECTION 1

#define SET_ANGLE 30.0f





















typedef struct
{
	fp32 max_pitch_angle;
	fp32 min_pitch_angle;
}gimbal_cali;








typedef struct
{
	fp32 calc_yaw_angle;
	fp32 calc_pitch_angle;
	
	int16_t give_yaw_current;
	int16_t give_pitch_current;
	int16_t give_shoot1_current;
  int16_t give_shoot2_current;
	
}gimbal_typedef;



typedef struct
{
	fp32 angle;
	fp32 rate;
	
}imu_data_feedback;

typedef struct
{
	float angle;
	float rate;
	
}motor_data_feedback;
typedef struct
{
	fp32 angle;
	
}imu_data_feedback_1;
typedef struct
{
	motor_data_feedback yaw_data;
	motor_data_feedback pitch_data;

	float yaw_error_angle;
	float pitch_error_angle;
}motor_typedef;

typedef struct
{
	const motor_measure_t *yaw_motor_data;
	const motor_measure_t *pitch_motor_data;
	const motor_measure_t *shoot1_motor_data;
	const motor_measure_t	*shoot2_motor_data;
	
	imu_data_feedback yaw_imu_ref_data;
	imu_data_feedback pitch_imu_ref_data;
	imu_data_feedback_1 yaw_imu_error_data;
	imu_data_feedback_1 pitch_imu_error_data;
	
	motor_typedef motor; 
	
}gimbal_feedback_typedef;


typedef struct
{
	const RC_ctrl_t *gimbal_ctrl;
	fp32 set_yaw_angle;
	fp32 set_pitch_angle;
	
}gimbal_rc_control_typedef;







//四个底盘电机的反馈值和拨盘电机反馈值
typedef struct
{
	const motor_measure_t *chassis_motor_data1;
	const motor_measure_t *chassis_motor_data2;
	const motor_measure_t *chassis_motor_data3;
	const motor_measure_t	*chassis_motor_data4;
	const motor_measure_t *chassis_motor_data5;
	
	int Vx_FeedBack;
	int Vy_FeedBack;
	int Wz_FeedBack;
	

}chassis_feedback_typedef;

//获取控制量
typedef struct
{
	const RC_ctrl_t *chassis_ctrl;
	int Vx;
	int Vy;
	int Wz;
	int triggle;
	
}chassis_rc_control_typedef;

typedef struct
{
	
	int W1;	
	int W2;
	int W3;
	int W4;
	int triggle;
	
}chassis_speed_set;


//计算得到的电流值
typedef struct
{
	int16_t give_motor1_current;
	int16_t give_motor2_current;
	int16_t give_motor3_current;
  int16_t give_motor4_current;
	int16_t give_trigge_current;
	
	int16_t give_vx;
	int16_t give_vy;
	int16_t give_wz;
}chassis_typedef;







extern RC_ctrl_t rc_ctrl;
extern imu_t imu;
extern uint8_t imu_rcv_buf[100];

extern pid_type_def yaw_speed_pid_ref,pitch_speed_pid_ref,yaw_position_pid_ref,pitch_position_pid_ref;
extern pid_type_def shoot1_pid_ref,shoot2_pid_ref;
extern pid_type_def Vx_ref,Vy_ref,Wz_ref;

extern pid_type_def trigger_angle,trigger_speed;



extern fp32 yaw_speed_pid[3],pitch_speed_pid[3],yaw_position_pid[3],pitch_position_pid[3],shoot1[3],shoot2[3];
extern fp32 Vx_pid[3],Vy_pid[3],Wz_pid[3];
extern fp32 trigge_speed_pid[3],trigge_angle_pid[3];





extern uint16_t this_shoot_time;
extern uint16_t last_shoot_time;
extern uint8_t shoot_flag;
extern fp32 error_angle;




extern uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];






extern fp32 data_W1[100],data_W2[100];
extern fp32 K_1;
extern fp32 K_2;
extern uint8_t this_current_time;
extern uint8_t last_current_time;
extern uint8_t interval;
extern fp32 Chassis_Pid[4][3];
extern pid_type_def wheel1,wheel2,wheel3,wheel4;
extern pid_type_def trigge;





