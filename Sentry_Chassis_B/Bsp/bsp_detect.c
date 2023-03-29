#include "bsp_detect.h"
#include "gpio.h"
#include "Chassis_Task.h"



uint8_t current_derection = 0;

/**
  * @brief 右边传感器
  */
void EXTI15_10_IRQHandler(void)
{
//	Chassis.Chassis_Mode_Flag.Current_Set_Direction  = RIGHT;
//	Chassis.Chassis_Mode_Flag.Current_Control_Mode = CHASSIS_BRAKE;
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
}


/**
  * @brief 左边红外传感器
  */
void EXTI9_5_IRQHandler(void)
{
//	Chassis.Chassis_Mode_Flag.Current_Set_Direction  = LEFT;
//	Chassis.Chassis_Mode_Flag.Current_Control_Mode = CHASSIS_BRAKE;
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);

}








