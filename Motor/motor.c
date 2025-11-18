/*
 * @File: motor.c
 * @Description: 
 * @Version: 1.0.0
 * @Author: 
 * @Date: 2023-09-13 20:33:36
 * @LastEditTime: 2023-09-15 15:45:48
 */

#include "motor.h"
#include "motor_task.h"
/**
 * @brief: 
 * @return {*}
 */
void motor_init(void)
{
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_Base_Start(&htim8);
	HAL_TIM_Base_Start(&htim9);
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);  //右后
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);  //右前
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);  //左前
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);  //左后
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);

	pid_init();
}



/*
	R0 ---TIM4.4->PD15	TIM4.3->PD14 	L0  ---TIM9.1->PE5	TIM9.2->PE6
	R1 ---TIM4.1->PD12	TIM4.2->PD13	L1  ---TIM8.3->PC8	TIM8.4->PC9
*/
void motor_set_pwm(uint8_t motor, int32_t pid_out)
{
	int32_t ccr = 0;
	
	if (pid_out >= 0)
	{
		if (pid_out > MOTOR_PWM_MAX)
			ccr = MOTOR_PWM_MAX;
		else
			ccr = pid_out;
		
		switch (motor)
		{	
			case 1: TIM9->CCR2 = 0; TIM9->CCR1 = ccr;	break;  //左前	L0
			case 2: TIM8->CCR3 = 0; TIM8->CCR4 = ccr;	break;  //左后	L1
			case 3: TIM4->CCR4 = 0; TIM4->CCR3 = ccr;	break;  //右前	R0
			case 4: TIM4->CCR2 = 0; TIM4->CCR1 = ccr;	break;  //右后	R1

			default: ; //TODO
		}
	}
	
	else if (pid_out < 0)
	{
		if (pid_out < -MOTOR_PWM_MAX)
			ccr = MOTOR_PWM_MAX;
		else
			ccr = -pid_out;
		
		switch (motor)
		{
			case 1: TIM9->CCR1 = 0; TIM9->CCR2 = ccr;	break;  //左前	L0
			case 2: TIM8->CCR4 = 0; TIM8->CCR3 = ccr;	break;  //左后	L1
			case 3: TIM4->CCR3 = 0; TIM4->CCR4 = ccr;	break;  //右前	R0
			case 4: TIM4->CCR1 = 0; TIM4->CCR2 = ccr;	break;  //右后	R1
			
			default: ; //TODO
		}
	}
}
