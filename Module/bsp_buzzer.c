/*
 * @File: bsp_buzzer.c
 * @Description: 
 * @Version: 1.0.0
 * @Author: 
 * @Date: 2023-09-13 20:33:36
 * @LastEditTime: 2023-09-15 15:36:41
 */
#include "bsp_buzzer.h"
#include "main.h" 

void buzzer_on(void)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
	vTaskDelay(5);
}

void buzzer_off(void)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
	vTaskDelay(5);
}
