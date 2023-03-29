#include "bsp_can.h"
#include "bsp_Referee.h"
#include "Referee_Behaviour.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
static uint32_t             send_mail_box;
static CAN_TxHeaderTypeDef  can_tx_message;
static CAN_TxHeaderTypeDef  can_rc_tx;
static uint8_t              can_rc_txdata[8];

extern ext_game_state_t game_state;

/**
  * @brief          can1��can2���˲�����ʼ��
  * @param[in]      none
  * @retval         none
  */
void can_filter_init(void)
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
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

}

void CanSendMessage(CAN_HandleTypeDef *hcan, uint32_t id, uint32_t dlc, uint8_t *message)
{
	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = dlc;
    
    if (can_tx_message.DLC > 8){
        can_tx_message.DLC = 8;
    }
    
    can_tx_message.StdId = id;
    
    HAL_CAN_AddTxMessage(hcan, &can_tx_message, message, &send_mail_box);
	}
}


//����ң������������
void Can_Send_RC_Data(CAN_HandleTypeDef *hcan,RC_ctrl_t *rc_data)
{
	
		//����ʱ��������������һ��Ϊ��ʱ���Ž��������ݷ��ͳ�ȥ
	  if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0)
		{
			uint32_t send_mail_box_h;
			can_rc_tx.StdId = 0x91;
			can_rc_tx.IDE = CAN_ID_STD;
			can_rc_tx.RTR = CAN_RTR_DATA;
			can_rc_tx.DLC = 8;
			//rc_left_channel_x
			can_rc_txdata[0] = rc_data->rc.ch[2]>>8;
			can_rc_txdata[1] = rc_data->rc.ch[2]; 
			//rc_left_channel_y
			can_rc_txdata[2] = rc_data->rc.ch[3]>>8;
			can_rc_txdata[3] = rc_data->rc.ch[3];
			//rc_up_channel
			can_rc_txdata[4] = rc_data->rc.ch[4]>>8;
			can_rc_txdata[5] = rc_data->rc.ch[4];
			
			can_rc_txdata[6] = rc_data->rc.s[1]<<4 | rc_data->rc.s[0];
			can_rc_txdata[7] = 0;
			
			//����״̬��־λ
			//bit0 ������ʼ״̬        0δ��ʼ 1��ʼ
			//bit1 �Ƿ�������ģʽ    0δ��ʼ 1��ʼ
			//bit2 �����ɨ��				   0δ��ʼ 1��ʼ
			
			//������ʼ
			if(game_state.game_progress == 4)
			{
				can_rc_txdata[7] |= 1;
			}
			else
			{
				can_rc_txdata[7] &= ~1;
			}
			
			
			
			//��Ϊ�췽�ڱ�
			if(robot_state.robot_id == 7)
			{
				//�췽ǰ��ս�ѱ����٣���������ģʽ
				if(game_robot_HP_t.red_outpost_HP == 0)
				{
					can_rc_txdata[7] |= 1<<1;
				}
				else 
				{
					can_rc_txdata[7] &= ~(1<<1);
				}
			}
			//��Ϊ�����ڱ�
			else if(robot_state.robot_id == 107)
			{
				//����ǰ��ս�ѱ����٣���������ģʽ
				if(game_robot_HP_t.blue_outpost_HP == 0)
				{
					can_rc_txdata[7] |= 1<<1;
				}
				else 
				{
					can_rc_txdata[7] &=~(1<<1);
				}
			}
			
			
			
			
      //��ǰʣ���ӵ�������
			if(bullet_remaining_t.bullet_remaining_num_17mm < 200)
			{
				can_rc_txdata[7] |= 1<<3;
			}
			else
			{
				can_rc_txdata[7] &=~(1<<3);
			}
    HAL_CAN_AddTxMessage(hcan, &can_rc_tx,can_rc_txdata, &send_mail_box);
		
		}
		
}