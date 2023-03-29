#include "imu.h"
#include "pid.h"
#include "variables.h"
#include "remote_control.h"
#include "bsp_can.h"




//遥控器数据
RC_ctrl_t rc_ctrl;



//pid计算结果
pid_type_def yaw_speed_pid_ref,pitch_speed_pid_ref,yaw_position_pid_ref,pitch_position_pid_ref,shoot1_pid_ref,shoot2_pid_ref;
pid_type_def wheel1,wheel2,wheel3,wheel4;
pid_type_def trigger_angle,trigger_speed;
pid_type_def Vx_ref,Vy_ref,Wz_ref;
pid_type_def Sever_ref;

fp32 yaw_speed_pid[3],pitch_speed_pid[3],yaw_position_pid[3],pitch_position_pid[3],shoot1[3],shoot2[3];
fp32 Vx_pid[3],Vy_pid[3],Wz_pid[3];
fp32 Chassis_Pid[4][3];
fp32 Sever_pid[3];
fp32 trigge_speed_pid[3],trigge_angle_pid[3];



fp32 data_W1[100],data_W2[100];
fp32 K_1 = 0;
fp32 K_2 = 0;




uint8_t last_current_time = 0;
uint8_t this_current_time = 0;
uint8_t interval = 0;




uint16_t this_shoot_time = 0;
uint16_t last_shoot_time = 0;
uint8_t shoot_flag = 0;
fp32 error_angle = 0;












