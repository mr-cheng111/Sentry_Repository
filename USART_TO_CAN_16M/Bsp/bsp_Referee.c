#include "bsp_Referee.h"
#include "usart.h"
#include "Referee_Behaviour.h"
#include "string.h"
#include "crc8_crc16.h"

uint8_t Referee_Buffer[2][Referee_Buffer_Length];
uint8_t Buffer_Choose = 0;
uint16_t RX_Length[2] = {0};

void usart1_init(uint8_t *Dst, uint32_t Buffer_Length)
{
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1,Dst,Buffer_Length);
}

void USART1_IRQHandler(void)
{
	if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)!= RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);	
		HAL_UART_IRQHandler(&huart1);
		HAL_UART_DMAStop(&huart1);//stop dma
	
		RX_Length[Buffer_Choose] = Referee_Buffer_Length - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);//get data length
		
		if(Buffer_Choose == 0)
		{
			Buffer_Choose = 1;
		}
		else Buffer_Choose = 0;

    }
	HAL_UART_Receive_DMA(&huart1,Referee_Buffer[Buffer_Choose],Referee_Buffer_Length);
}

void USART_Data_Slove(void)
{
	for(uint16_t i = 0; i < RX_Length[0] ; i ++)
	{
		if(Referee_Buffer[0][i] == 0xA5)
		{
			referee_data_solve(&Referee_Buffer[0][i]);
		}
	}
	memset(Referee_Buffer[0],0,RX_Length[0]);
	RX_Length[0] = 0;
	
	for(uint16_t i = 0; i < RX_Length[1] ; i ++)
	{
		if(Referee_Buffer[1][i] == 0xA5)
		{
			referee_data_solve(&Referee_Buffer[1][i]);
		}
	}
	memset(Referee_Buffer[1],0,RX_Length[1]);
	RX_Length[1] = 0;
	

}
