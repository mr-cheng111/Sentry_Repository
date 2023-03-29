#include "bsp_motor.h"
#include "Data_Tx_Task.h"
#include "can.h"

//can数据解包
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
        (ptr)->angle = (ptr)->ecd / 8191.0f * 360.0f - 180.0f;          \
    }

void process_motor_measure(motor_measure_t *motor_measure);    

motor_measure_t can1_motor[8];
motor_measure_t can2_motor[8];

static CAN_TxHeaderTypeDef  can_tx_message_h;
static uint8_t              can_send_data_h[8];
static CAN_TxHeaderTypeDef  can_tx_message_l;
static uint8_t              can_send_data_l[8];



void can_filter_init(void);
void CAN1_CMD(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4, int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);
void CAN2_CMD(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4, int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);
const motor_measure_t *get_can1_motor_measure_point(uint8_t i);
const motor_measure_t *get_can2_motor_measure_point(uint8_t i);
void get_shoot_ref_value(fp32 *data);


/**
  * @brief          电机过零点计数
  * @param[in]      电机传感器值结构体变量
  * @retval         none
  */    
void process_motor_measure(motor_measure_t *motor_measure)
{
    if ((motor_measure->ecd - motor_measure->last_ecd) > 6400)
    {
        motor_measure->rounds++;
    }
    else if ((motor_measure->ecd - motor_measure->last_ecd) < -6400)
    {
        motor_measure->rounds--;
    }
    
    motor_measure->series_ecd = motor_measure->ecd + 8192* motor_measure->rounds;
    motor_measure->series_angle = motor_measure->angle  + 360* motor_measure->rounds;
}



void chassis_message_process(uint8_t *message, shoot_referee_t *referee)
{		
    referee->cooling_heat = (uint16_t)(message[2] << 8 | message[3]);
    referee->current_heat = (uint16_t)(message[4] << 8 | message[5]);
    referee->current_rate = (fp32)((fp32) message[7] * 30.0f / 255.0f);
}


/**
  * @brief          can1发送电机控制电流(0x200下的0x201,0x202,0x203,0x204; 0x1FF下的0x205,0x206,0x207,0x208)
  * @param[in]      can1下8个电机的供给电流控制，范围：3508:[-16384,16384]; 6020:[-30000,30000]; 2006:[-10000,10000];
  * @retval         none
  */
void CAN1_CMD(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4, int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
    uint32_t send_mail_box_l;
    can_tx_message_l.StdId = CAN_ALL_ID_L;
    can_tx_message_l.IDE = CAN_ID_STD;
    can_tx_message_l.RTR = CAN_RTR_DATA;
    can_tx_message_l.DLC = 0x08;
    can_send_data_l[0] = motor1 >> 8;
    can_send_data_l[1] = motor1;
    can_send_data_l[2] = motor2 >> 8;
    can_send_data_l[3] = motor2;
    can_send_data_l[4] = motor3 >> 8;
    can_send_data_l[5] = motor3;
    can_send_data_l[6] = motor4 >> 8;
    can_send_data_l[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &can_tx_message_l, can_send_data_l, &send_mail_box_l);
    
    uint32_t send_mail_box_h;
    can_tx_message_h.StdId = CAN_ALL_ID_H;
    can_tx_message_h.IDE = CAN_ID_STD;
    can_tx_message_h.RTR = CAN_RTR_DATA;
    can_tx_message_h.DLC = 0x08;
    can_send_data_h[0] = (motor5 >> 8);
    can_send_data_h[1] = motor5;
    can_send_data_h[2] = (motor6 >> 8);
    can_send_data_h[3] = motor6;
    can_send_data_h[4] = (motor7 >> 8);
    can_send_data_h[5] = motor7;
    can_send_data_h[6] = (motor8 >> 8);
    can_send_data_h[7] = motor8;
    
    HAL_CAN_AddTxMessage(&hcan1, &can_tx_message_h, can_send_data_h, &send_mail_box_h);
}

/**
  * @brief          can2发送电机控制电流(0x200下的0x201,0x202,0x203,0x204; 0x1FF下的0x205,0x206,0x207,0x208)
  * @param[in]      can2下8个电机的供给电流控制，范围：3508:[-16384,16384]; 6020:[-30000,30000]; 2006:[-10000,10000];
  * @retval         none
  */
void CAN2_CMD(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4, int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
    uint32_t send_mail_box_l;
    can_tx_message_l.StdId = CAN_ALL_ID_L;
    can_tx_message_l.IDE = CAN_ID_STD;
    can_tx_message_l.RTR = CAN_RTR_DATA;
    can_tx_message_l.DLC = 0x08;
    can_send_data_l[0] = motor1 >> 8;
    can_send_data_l[1] = motor1;
    can_send_data_l[2] = motor2 >> 8;
    can_send_data_l[3] = motor2;
    can_send_data_l[4] = motor3 >> 8;
    can_send_data_l[5] = motor3;
    can_send_data_l[6] = motor4 >> 8;
    can_send_data_l[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan2, &can_tx_message_l, can_send_data_l, &send_mail_box_l);
    
    uint32_t send_mail_box_h;
    can_tx_message_h.StdId = CAN_ALL_ID_H;
    can_tx_message_h.IDE = CAN_ID_STD;
    can_tx_message_h.RTR = CAN_RTR_DATA;
    can_tx_message_h.DLC = 0x08;
    can_send_data_h[0] = (motor5 >> 8);
    can_send_data_h[1] = motor5;
    can_send_data_h[2] = (motor6 >> 8);
    can_send_data_h[3] = motor6;
    can_send_data_h[4] = (motor7 >> 8);
    can_send_data_h[5] = motor7;
    can_send_data_h[6] = (motor8 >> 8);
    can_send_data_h[7] = motor8;
    
    HAL_CAN_AddTxMessage(&hcan2, &can_tx_message_h, can_send_data_h, &send_mail_box_h);
}



/**
  * @brief          返回can1电机数据指针
  * @param[in]      i: 电机编号,范围[0,7]
  * @retval         电机数据指针
  */
const motor_measure_t *get_can1_motor_measure_point(uint8_t i)
{
    return &can1_motor[(i & 0x07)];
}

/**
  * @brief          返回can2电机数据指针
  * @param[in]      i: 电机编号,范围[0,7]
  * @retval         电机数据指针
  */
const motor_measure_t *get_can2_motor_measure_point(uint8_t i)
{
    return &can2_motor[(i & 0x07)];
}


