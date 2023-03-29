#include "Debug_Task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_led.h"
#include "Referee_Behaviour.h"
#include "usart.h"
#include "stdio.h"

extern ext_power_heat_data_t power_heat_data_t;

//#define printf(...)			HAL_UART_Transmit(&huart3,\
//                                         (uint8_t *)u1_buf,\
//                                         sprintf((char*)u1_buf,__VA_ARGS__),0x01)			
//uint8_t u1_buf[11];

void Debug_Task_Start(void const * argument)
{
  for(;;)
  {

//		printf("%f\r\n",power_heat_data_t.chassis_power);
//		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_9);
    osDelay(1);
  }
}





