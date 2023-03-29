#ifndef _SENTRY7_KEY_MAP_
#define _SENTRY7_KEY_MAP_

#include "Remote.h"
#define YAW_REMOTE_SENS                         0.25f
#define PITCH_REMOTE_SENS                       0.25f
#define YAW_MOUSE_SENS                          100
#define PITCH_MOUSE_SENS                        50

// ��̨�˶�����ָ��
// YAW
#define GIMBAL_CMD_YAW_KEYMAP           NormalizedLimit(MouseMoveY()*YAW_MOUSE_SENS + RemoteChannalLeftY()*YAW_REMOTE_SENS)
// PITCH
#define GIMBAL_CMD_PITCH_KEYMAP         -NormalizedLimit(MouseMoveX()*PITCH_MOUSE_SENS + RemoteChannalLeftX()*PITCH_REMOTE_SENS)

// �����ջ�����
#define HEAT_CLOSED_LOOP_SWITCH_KEYMAP  CheakKeyPressOnce(KEY_PRESSED_OFFSET_B)
// ��Ƶ����
// ������Ƶ
#define SHOOT_FREQ_RAISE_KEYMAP         CheakKeyPressOnce(KEY_PRESSED_OFFSET_E)
// ������Ƶ
#define SHOOT_FREQ_REDUCE_KEYMAP        CheakKeyPressOnce(KEY_PRESSED_OFFSET_Q)



// ״̬������
// ��̨����
//#define GIMBAL_WEAK_KEYMAP              SwitchRightDownSide()
// ��̨ʹ��
#define GIMBAL_ENABLE_KEYMAP            SwitchRightMidSide() || SwitchRightUpSide()
// �������ʹ��
#define SHOOTER_ENABLE_KEYMAP           SwitchRightUpSide()
// ����ʹ��
#define CHASSIS_ENABLE_KEYMAP           SwitchLeftDownSide()


// ����ָ��
#define SHOOT_COMMAND_KEYMAP            ((RemoteDial() == 1.0f) || (MousePressLeft()))
// ����ָ��
#define AIMBOT_COMMAND_KEYMAP           (SwitchLeftUpSide() || MousePressRight())





#endif
