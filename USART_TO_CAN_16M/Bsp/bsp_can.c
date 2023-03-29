#include "bsp_can.h"
#include "bsp_Referee.h"
//#include "Referee_Behaviour.h"

extern CAN_HandleTypeDef hcan;
static uint32_t             send_mail_box;
static CAN_TxHeaderTypeDef  can_tx_message;
       uint8_t              can_tx_data[8];
motor_measure_t can_motor[4];

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
        (ptr)->angle = (ptr)->ecd / 8192.0f * 360.0f - 180.0f;          \
    }

#define CAN_Motor1_ID 0x201
#define CAN_Motor2_ID 0x202
#define CAN_Motor3_ID 0x203
#define CAN_Motor4_ID 0x204
	
	
void process_motor_measure(motor_measure_t *motor_measure);
/**
  * @brief          can的滤波器初始化
  * @param[in]      none
  * @retval         none
  */
void Can_Filter_Init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
    HAL_CAN_ConfigFilter(&hcan, &can_filter_st);
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

}

void CanSendMessage(CAN_HandleTypeDef *hcan, uint32_t id, uint32_t dlc, uint8_t *message)
{
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = dlc;
    
    if (can_tx_message.DLC > 8)
	{
        can_tx_message.DLC = 8;
    }
    can_tx_message.StdId = id;
    
    HAL_CAN_AddTxMessage(hcan, &can_tx_message, message, &send_mail_box);
}


//发送数据
void Can_Send_Data(CAN_HandleTypeDef *hcan,uint16_t frame,uint8_t Data_Length,uint8_t *data)
{
		//当此时发送邮箱至少有一个为空时，才将控制数据发送出去
	  if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
		{
			uint32_t send_mail_box_h;
			can_tx_message.StdId = frame;
			can_tx_message.IDE = CAN_ID_STD;
			can_tx_message.RTR = CAN_RTR_DATA;
			can_tx_message.DLC = Data_Length;
			can_tx_data[0] = *(data+0);
			can_tx_data[1] = *(data+1); 
			can_tx_data[2] = *(data+2);
			can_tx_data[3] = *(data+3);
			can_tx_data[4] = *(data+4);
			can_tx_data[5] = *(data+5);
			can_tx_data[6] = *(data+6);
			can_tx_data[7] = *(data+7);
			HAL_CAN_AddTxMessage(hcan, &can_tx_message,can_tx_data, &send_mail_box);
		}
		
}
/**
  * @brief          hal库CAN1 fifo1回调函数,接收电机数据
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
        {
            static uint8_t i = 0;
  			i = rx_header.StdId - CAN_Motor1_ID;
            get_motor_measure(&can_motor[i], rx_data);
            process_motor_measure(&can_motor[i]);
			
			
           
            break;
        }
        default:
        {
            break;
        }
    }
}



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





