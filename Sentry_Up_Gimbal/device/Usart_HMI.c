#include "Usart_HMI.h"
#include "Setting.h"
#include "AimbotCan.h"
#include "CalculateThread.h"
#include "RefereeCan.H"

#define Usart_HMI_BUFFER_SIZE 10

uint8_t Usart_HMI_Buffer[Usart_HMI_BUFFER_SIZE] = {0};

float Lkp = 20;
float Rkp = 18;
uint16_t SetSpeed = 7300;

#define printf(...)			HAL_UART_Transmit(&huart1,\
                        (uint8_t *)u1_buf,\
                        sprintf((char*)u1_buf,__VA_ARGS__),0xff)			
uint8_t u1_buf[37];

void USART1_IRQHandler(void)
{
	if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)!= RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);	
		HAL_UART_IRQHandler(&huart1);
		HAL_UART_DMAStop(&huart1);//stop dma
    uint8_t RX_Length = Usart_HMI_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);//get data length
		
     for (uint8_t i = 0; i < RX_Length; i++)
		 {
      if (Usart_HMI_Buffer[i] == 0x65)
      {
				Usart_HMI_Unpacked(&Usart_HMI_Buffer[i]);
      }
     }
    }
	HAL_UART_Receive_DMA(&huart1, Usart_HMI_Buffer,100);
}


void Usart_HMI_init(void)
{
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1, Usart_HMI_Buffer, Usart_HMI_BUFFER_SIZE);
}

void Usart_HMI_Unpacked(uint8_t* Point)
{
	//Ò³Ãæ1
	if(*(Point+1) == 0x01)
	{
		if(*(Point+2) == 0x02)//+50
		{
			SetSpeed+=50;
		}
		if(*(Point+2) == 0x03)//-50
		{
			SetSpeed-=50;			
		}
		if(*(Point+2) == 0x05)//+100
		{
			SetSpeed+=100;
			
		}
		if(*(Point+2) == 0x04)//-100
		{
			SetSpeed-=100;
		}
	}
	//Ò³Ãæ2
	if(*(Point+1) == 0x02)
	{
		if(*(Point+2) == 0x01)//L_KP+0.5
		{
			Lkp+=0.5;
		}
		if(*(Point+2) == 0x02)//L_KP-0.5
		{
			Lkp-=0.5;
		}
		if(*(Point+2) == 0x04)//R_KP+0.5
		{
			Rkp+=0.5;
		}
		if(*(Point+2) == 0x03)//R_KP-0.5
		{
			Rkp-=0.5;
		}
	}
}

uint8_t ReWriteHMI(char *L_KP,char *R_KP,char *SET_SPEED,char *FeedBack_SPEED)
{
//	printf("LEFT_KP.val=%c%c%c%c",*L_KP,*(L_KP+1),*(L_KP+2),*(L_KP+3));
//  printf("\xff\xff\xff");
//	printf("RIGHT_KP.val=%c%c%c%c",*R_KP,*(R_KP+1),*(R_KP+2),*(R_KP+3));
//	printf("\xff\xff\xff");
//	printf("Shoot_SetSpeed.val=%c%c%c%c",*SET_SPEED,*(SET_SPEED+1),*(SET_SPEED+2),*(SET_SPEED+3));
//	printf("\xff\xff\xff");
//  printf("Shoot_Ref.val=%c%c%c%c",*FeedBack_SPEED,*(FeedBack_SPEED+1),*(FeedBack_SPEED+2),*(FeedBack_SPEED+3));
//	printf("\xff\xff\xff");

	printf("%f,%f\n",Gimbal.Imu.PitchAngle,Gimbal.Command.Pitch);

	return 1;
	
}












