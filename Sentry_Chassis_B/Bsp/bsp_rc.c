#include "bsp_rc.h"
#include "usart.h"
#include "string.h"
#include "bsp_can.h"
//遥控器数据
RC_ctrl_t rc_ctrl;

extern DMA_HandleTypeDef hdma_uart5_rx;

//遥控器接收dma缓存区
uint8_t sbus_buf[SBUS_BUFF_SIZE];

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart5.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_uart5_rx);
    while(hdma_uart5_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_uart5_rx);
    }
    hdma_uart5_rx.Instance->PAR = (uint32_t) & (UART5->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_uart5_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_uart5_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    hdma_uart5_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_uart5_rx.Instance->CR, DMA_SxCR_DBM);
    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_uart5_rx);


}

void RC_unable(void)
{
    __HAL_UART_DISABLE(&huart5);
}


void RC_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart5);
    __HAL_DMA_DISABLE(&hdma_uart5_rx);

    hdma_uart5_rx.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_uart5_rx);
    __HAL_UART_ENABLE(&huart5);

}
void remote_init(void)
{
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart5, sbus_buf, SBUS_BUFF_SIZE);
}


/**
  * @brief          遥控器协议解析
  * @param[in]      none
  * @param[out]     none
  * @retval         none
  */
void sbus_process(void)
{
    
    rc_ctrl.rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl.rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl.rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl.rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    
    rc_ctrl.rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl.rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    
    rc_ctrl.mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl.mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl.mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    
    rc_ctrl.mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl.mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    
    rc_ctrl.key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    
    rc_ctrl.rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl.rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[4] -= RC_CH_VALUE_OFFSET;
    
}



void UART5_IRQHandler(void)
{
	__HAL_UART_DISABLE_IT(&huart5, UART_IT_ERR);
	__HAL_UART_DISABLE_IT(&huart5, UART_IT_LBD);
	if(__HAL_UART_GET_FLAG(&huart5,UART_FLAG_IDLE) != RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart5);	
		HAL_UART_AbortReceive(&huart5);//关闭dma
		
		uint16_t RX_Length = SBUS_BUFF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart5_rx);//计算数据长度
		if(RX_Length == RC_FRAME_LENGTH)
		{
			sbus_process();
		}
	}
	HAL_UART_Receive_DMA(&huart5, sbus_buf, SBUS_BUFF_SIZE);//重启DMA
}



RC_ctrl_t *get_remote_control_point(void)
{
	return &rc_ctrl;
}

