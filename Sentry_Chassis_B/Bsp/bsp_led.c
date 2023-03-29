#include "bsp_led.h"
#include "main.h"

void Led_Show(void)
{
		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_8);
		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_9);
		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_10);
		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_11);
}


