#include "struct_typedef.h"

#ifndef BSP_IMU_H
#define BSP_IMU_H

#define IMU_BUFF_SIZE 100 
#define IMU_PACK_LENGTH 11


typedef struct
{
    fp32    yaw;
    fp32    pitch;
    fp32    row;
} imu_value_t;

typedef struct
{
    imu_value_t angle;
    imu_value_t rate;
} imu_t;
#endif

extern imu_t imu;
extern uint8_t imu_rcv_buf[100];

extern void imu_init(void);
extern void imu_rate_process(uint8_t i,uint8_t *rcv);
extern void imu_angle_process(uint8_t i,uint8_t *rcv);
















