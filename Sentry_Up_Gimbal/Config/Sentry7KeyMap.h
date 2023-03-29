#ifndef _SENTRY7_KEY_MAP_
#define _SENTRY7_KEY_MAP_

#include "Remote.h"
#define YAW_REMOTE_SENS                         0.25f
#define PITCH_REMOTE_SENS                       0.25f
#define YAW_MOUSE_SENS                          100
#define PITCH_MOUSE_SENS                        50

// 云台运动控制指令
// YAW
#define GIMBAL_CMD_YAW_KEYMAP           NormalizedLimit(MouseMoveY()*YAW_MOUSE_SENS + RemoteChannalLeftY()*YAW_REMOTE_SENS)
// PITCH
#define GIMBAL_CMD_PITCH_KEYMAP         -NormalizedLimit(MouseMoveX()*PITCH_MOUSE_SENS + RemoteChannalLeftX()*PITCH_REMOTE_SENS)

// 热量闭环开关
#define HEAT_CLOSED_LOOP_SWITCH_KEYMAP  CheakKeyPressOnce(KEY_PRESSED_OFFSET_B)
// 射频调节
// 增加射频
#define SHOOT_FREQ_RAISE_KEYMAP         CheakKeyPressOnce(KEY_PRESSED_OFFSET_E)
// 降低射频
#define SHOOT_FREQ_REDUCE_KEYMAP        CheakKeyPressOnce(KEY_PRESSED_OFFSET_Q)



// 状态机设置
// 云台无力
//#define GIMBAL_WEAK_KEYMAP              SwitchRightDownSide()
// 云台使能
#define GIMBAL_ENABLE_KEYMAP            SwitchRightMidSide() || SwitchRightUpSide()
// 发射机构使能
#define SHOOTER_ENABLE_KEYMAP           SwitchRightUpSide()
// 底盘使能
#define CHASSIS_ENABLE_KEYMAP           SwitchLeftDownSide()


// 发弹指令
#define SHOOT_COMMAND_KEYMAP            ((RemoteDial() == 1.0f) || (MousePressLeft()))
// 自瞄指令
#define AIMBOT_COMMAND_KEYMAP           (SwitchLeftUpSide() || MousePressRight())





#endif
