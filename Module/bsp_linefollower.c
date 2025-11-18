#include "bsp_linefollower.h"
#include "usart.h"
#include "tim.h"
#include "scaner.h"
#include "stdio.h"
#include "main.h"
#include  "FreeRTOS.h"
#include  "task.h"


uint8_t	infrare_open = 0;
volatile struct Infrared_Sensor infrared;

/*红外读值 - 碰到障碍物时为1*/
void get_Infrared(void)
{
	infrared.head_right = !(uint8_t)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12);
	infrared.head_left = !(uint8_t)HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9);
}







