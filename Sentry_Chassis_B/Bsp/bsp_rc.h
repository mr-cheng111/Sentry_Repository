#ifndef BSP_RC_H
#define BSP_RC_H
#include "struct_typedef.h"
#include "main.h"

//遥控器帧间隔时间上限(ms)
#define RC_CHANNAL_ERROR_VALUE 700
#define RC_FRAME_LENGTH 18u
#define SBUS_BUFF_SIZE  100u




#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint8_t)1)
#define RC_SW_MID               ((uint8_t)3)
#define RC_SW_DOWN              ((uint8_t)2)
#define RC_NO_CONNECT           ((uint8_t)0)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */

typedef __PACKED_STRUCT
{
        __PACKED_STRUCT
        {
                int16_t ch[5];
                char s[2];
        } rc;
        __PACKED_STRUCT
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __PACKED_STRUCT
        {
                uint16_t v;
        } key;

} RC_ctrl_t;

/* ----------------------- Internal Data ----------------------------------- */

extern void remote_init(void);
extern RC_ctrl_t *get_remote_control_point(void);
#endif
