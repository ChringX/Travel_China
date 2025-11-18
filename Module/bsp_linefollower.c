/*
 * @File: bsp_linefollower.c
 * @Description: 
 * @Version: 1.0.0
 * @Author: 
 * @Date: 2023-09-13 20:33:36
 * @LastEditTime: 2023-09-15 15:36:53
 */
#include "bsp_linefollower.h"
#include "usart.h"
#include "tim.h"
#include "scaner.h"
#include "stdio.h"
#include "main.h"
#include  "FreeRTOS.h"
#include  "task.h"
//此文件用于红外走
uint8_t	infrare_open=0;

volatile struct Infrared_Sensor infrared;


void get_Infrared(void){     //碰到障碍物时为1
	
	infrared.head_right = !(uint8_t)HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9);
	infrared.tail_right = !(uint8_t)HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8);
	infrared.inside_outside_right = !(uint8_t)HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_4);
	
	infrared.head_left = !(uint8_t)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13);
	infrared.tail_left = !(uint8_t)HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_10);
	infrared.inside_outside_left = !(uint8_t)HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_11);
}







