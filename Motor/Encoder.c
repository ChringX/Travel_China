/*
 * @File: Encoder.c
 * @Description: 
 * @Version: 1.0.0
 * @Author: 
 * @Date: 2023-09-13 20:33:36
 * @LastEditTime: 2023-09-15 15:48:27
 */
#include "encoder.h"
#include  <stdlib.h>
#include <string.h>
#include "uart.h"
#include "motor_task.h"
#include "speed_ctrl.h"
wheel wheel_1,wheel_2,wheel_3,wheel_4;

/*************************
	电机编码器线数13
	电机减速比144
	轮子直径104mm
*************************/
#define coefficient 0.000909090909f
#define Half_ARR 32767
uint8_t dog[4] = {0};   //电机狗

short Buffer_Encoder[4] = {0};  
short Speed[4];

/**
 * @brief: 定义小车的轮子编码器属于哪个定时器
 * @return {*}
 */
void Encoder_init(void){
	//左前
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim1);
	
	//左后
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim2);

	//右前
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim3);
	
	//右后
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim5);
	
	HAL_TIM_Base_Start_IT(&htim11);
}


/**
 * @brief: 
 * @param {TIM_HandleTypeDef} *htim
 * @return {*}
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM14) 
	{
		HAL_IncTick();
	}
	if(htim->Instance == TIM1) 
	{
		
	}
	if(htim->Instance == TIM3) 
	{
		
	}
	if(htim->Instance == TIM2) 
	{
		
	}
	if(htim->Instance == TIM5) 
	{
		
	}

}

/*重新计算脉冲*/
void encoder_clear(void)
{
	motor_all.Distance = 0;
	
	TIM1->CNT=0;
	TIM3->CNT=0;
	TIM2->CNT=0;
	TIM5->CNT=0;
}
