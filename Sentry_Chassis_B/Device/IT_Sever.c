#include "bsp_motor.h"
#include "Data_Tx_Task.h"
#include "can.h"

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
        (ptr)->angle = (ptr)->ecd / 8192.0f * 360.0f - 180.0f;          \
    }

extern motor_measure_t can1_motor[8];
extern motor_measure_t can2_motor[8];
void process_motor_measure(motor_measure_t *motor_measure);
/**
  * @brief          hal库CAN1 fifo0回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    

    switch (rx_header.StdId)
    {
        case CAN_Motor1_ID:
        case CAN_Motor2_ID:
        case CAN_Motor3_ID:
        case CAN_Motor4_ID:
        case CAN_Motor5_ID:
        case CAN_Motor6_ID:
        case CAN_Motor7_ID:
        case CAN_Motor8_ID:
        {
            static uint8_t i = 0;
  					i = rx_header.StdId - CAN_Motor1_ID;
            get_motor_measure(&can1_motor[i], rx_data);
            process_motor_measure(&can1_motor[i]);
            
            break;
        }
        default:
        {
            break;
        }
    }
}
/**
  * @brief          hal库CAN2 fifo1回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
    switch (rx_header.StdId)
    {
        case CAN_Motor1_ID:
        case CAN_Motor2_ID:
        case CAN_Motor3_ID:
        case CAN_Motor4_ID:
        case CAN_Motor5_ID:
        case CAN_Motor6_ID:
        case CAN_Motor7_ID:
        case CAN_Motor8_ID:
        {
            static uint8_t i = 0;
  					i = rx_header.StdId - CAN_Motor1_ID;
            get_motor_measure(&can2_motor[i], rx_data);
            process_motor_measure(&can2_motor[i]);
            
            break;
        }      
        default:
        {
            break;
        }
    }
}

























