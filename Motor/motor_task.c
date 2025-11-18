/*
 * @File: motor_task.c
 * @Description: 电机任务
 * @Version: 1.0.0
 * @Author:
 * @Date: 2023-09-13 20:33:36
 * @LastEditTime: 2023-09-15 15:46:37
 */
#include "motor_task.h"
#include "encoder.h"
#include "motor.h"
#include "uart.h"
#include "speed_ctrl.h"
#include "pid.h"
#include "turn.h"
#include "scaner.h"
#include "bsp_linefollower.h"
#include "sin_generate.h"
#include "bsp_buzzer.h"
#include "openmv.h"
#include "map.h"
#include "QR.h"
#include "delay.h"
#include "bsp_led.h"
#include "math.h"
#include "barrier.h"
uint8_t ScanMode;
TaskHandle_t motor_handler;
int dirct[4] = {-1, 1, -1, -1};
volatile uint8_t PIDMode;
uint8_t line_gyro_switch = 0;
#define Speed_Bias_Up 10
#define Speed_Bias_Down 10
#define measure_k 1.165f
int motor1;
int motor_time = 0;

/**
 * @brief:
 * @param {void} *pvParameters
 * @return {*}
 */
void motor_task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount(); // 获取系统节拍
	static uint8_t mouse = 0;			 // 小灯鼠
	static uint8_t scaner_time = 0;
	
	while (1)
	{
		scaner_time++;
		if (PIDMode == is_Line)
		{
			getline_error();
		}
		if(scaner_time == 5)
		{
			get_motor_speed();
			motor_all.encoder_avg = (motor_L0.measure + motor_L1.measure + motor_R0.measure + motor_R1.measure) / 4;
			motor_all.Distance += motor_all.encoder_avg;
	
			if (infrare_open)
			{ // 红外任务
				get_Infrared();
			}
	
			// 陀螺仪自平衡->循迹
			if (line_gyro_switch == 1) // 这里的line_gyro_switch是在PIDMODE切换情况下所产生的标志位
			{
				line_pid_obj = gyroG_pid;
				TC_speed = TG_speed;
				gyroG_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
				TG_speed = (struct Gradual){0, 0, 0};
				line_gyro_switch = 0;
			}
			else if (line_gyro_switch == 2)
			{
				gyroG_pid = line_pid_obj;
				TG_speed = TC_speed;
				line_pid_obj = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
				TC_speed = (struct Gradual){0, 0, 0};
				line_gyro_switch = 0;
			}
			else
			{
				if (PIDMode == is_Line)
				{
					gradual_cal(&TC_speed, motor_all.Cspeed, motor_all.Cincrement,motor_all.CDOWNincrement);
					Go_Line(TC_speed.Now);
				}
				else
				{
					motor_all.Cspeed = 0;
				}
	
				// 转弯PID控制
				if (PIDMode == is_Turn)
				{
					if (nodesr.nowNode.function == UpStage || nodesr.nowNode.function == View || nodesr.nowNode.function == View1 || nodesr.nowNode.function == BSoutPole || nodesr.nowNode.function == BHM)
					{
						if (Stage_turn_Angle(angle.AngleT))
						{
							gyroT_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0};
						}
					}
					else if (Turn_Angle(angle.AngleT))
					{
						gyroT_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0};
					}
				}
	
				// 自平衡PID控制
				if (PIDMode == is_Gyro)
				{
					gradual_cal(&TG_speed, motor_all.Gspeed, motor_all.Gincrement,motor_all.GDOWNincrement);
					runWithAngle(angle.AngleG, TG_speed.Now);
				}
				else
				{
					motor_all.Gspeed = 0;
				}
			}
	
			mouse++;
			if (mouse > 100)
			{
				mouse = 0;
				LED_C1_Toggle();
			}
	
			if(DownLiuShui)
			{
				motor_L0.target = motor_all.Lspeed * 1.3f; //1.45
				motor_L1.target = motor_all.Lspeed;
				motor_R0.target = motor_all.Rspeed * 1.3f;
				motor_R1.target = motor_all.Rspeed;
			}
			else if(L_Fight)
			{
				motor_L0.target = motor_L1.target = motor_all.Lspeed * 1.05f;
				motor_R0.target = motor_R1.target = motor_all.Rspeed;
			}
			else if(R_Fight)
			{
				motor_L0.target = motor_L1.target = motor_all.Lspeed;
				motor_R0.target = motor_R1.target = motor_all.Rspeed * 1.05f;
			}
			else
			{
				motor_L0.target = motor_L1.target = motor_all.Lspeed;
				motor_R0.target = motor_R1.target = motor_all.Rspeed;
			}
//			motor_L0.target = motor_L1.target = 25;
//			motor_R0.target = motor_R1.target = 25;
			
			incremental_PID(&motor_L0, &motor_pid_paramL0);
			incremental_PID(&motor_L1, &motor_pid_paramL1);
			incremental_PID(&motor_R0, &motor_pid_paramR0);
			incremental_PID(&motor_R1, &motor_pid_paramR1);
	
			if (motor_all.Lspeed == 0 && motor_all.Rspeed == 0 && ((motor_L0.measure + motor_L1.measure + motor_R0.measure + motor_R1.measure) <= 3)) // 看门时速度和低于50输出0
			{
				motor_pid_clear();
			}
	
			if (PIDMode == is_Free) // 判断是否要自控
			{
			}
			else
			{
//				if(motor_all.Lspeed != 0 || motor_all.Rspeed != 0)
//				{
//					if(motor_L0.measure == 0 || motor_L1.measure == 0 || motor_R0.measure == 0 || motor_R1.measure == 0)
//					{
//						motor_time++;
//						if(motor_time >= 40)
//						{
//							while(1)
//							{
//								motor_set_pwm(1, 0);
//								motor_set_pwm(2, 0);
//								motor_set_pwm(3, 0);
//								motor_set_pwm(4, 0);
//								buzzer_on();
//								delay_ms(500);
//								buzzer_off();
//								delay_ms(500);
//							}
//						}
//					}
//					else
//					{
//						motor_time = 0;
//					}
//				}
	
				if(ScanMode == is_Front)
				{
					motor_set_pwm(1, (int32_t)motor_L0.output);
					motor_set_pwm(2, (int32_t)motor_L1.output);
					motor_set_pwm(3, (int32_t)motor_R0.output);
					motor_set_pwm(4, (int32_t)motor_R1.output);
				}
				else
				{
					motor_set_pwm(4, -(int32_t)motor_L0.output);
					motor_set_pwm(3, -(int32_t)motor_L1.output);
					motor_set_pwm(2, -(int32_t)motor_R0.output);
					motor_set_pwm(1, -(int32_t)motor_R1.output);
				}
			}
//			printf("%f\r\n",getAngleZ());
//			printf("%f,    %f,   %f,    %f,    %d\r\n",motor_L0.target,motor_L0.measure,motor_R0.target,motor_R0.measure,ScanMode);
			//		printf("%f,%f,%f,%f\r\n",imu.yaw,motor_all.Lspeed,motor_all.Rspeed,angle.AngleG);
			//		printf("%f,%f,%f,%f\r\n",motor_L0.measure,motor_L1.measure,motor_R0.measure,motor_R1.measure);
			//		printf("%f,%f\r\n",motor_R1.target,motor_R1.measure);
			scaner_time = 0;
		}
		vTaskDelayUntil(&xLastWakeTime, (1 / portTICK_RATE_MS)); // 绝对休眠5ms // INCLUDE_vTaskDelayUntil 1
	}
}

/**
 * @brief: PID任务创建
 * @return {*}
 */
void motor_task_create(void)
{
	xTaskCreate((TaskFunction_t)motor_task,		  // 任务函数
				(const char *)"motor_task",		  // 任务名字
				(uint32_t)motor_size,			  // 任务堆栈大小
				(void *)NULL,					  // 传递给任务参数的指针参数
				(UBaseType_t)motor_task_priority, // 任务的优先级
				(TaskHandle_t *)&motor_handler);  // 任务句柄
}

/**
 * @brief: 
 * @param {uint8_t} target_mode
 * @return {*}
 */
void pid_mode_switch(uint8_t target_mode)
{
	switch (target_mode)
	{
	case is_Turn:
	{
		gyroG_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
		TG_speed = (struct Gradual){0, 0, 0};
		line_pid_obj = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
		TC_speed = (struct Gradual){0, 0, 0};
		gyroT_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
		break;
	}

	case is_Line:
	{
		if (PIDMode == is_Gyro) // 从自平衡切换到循线
		{
			line_gyro_switch = 1;
		}
		else
		{
			gyroT_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
		}
		break;
	}

	case is_Gyro:
	{
		if (PIDMode == is_Line) // 从循线切换到自平衡
		{
			line_gyro_switch = 2;
		}
		else
		{
			gyroT_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
		}
		break;
	}

	case is_Free:
	{
		break;
	}

	case is_No:
	{
		line_pid_obj = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
		TC_speed = (struct Gradual){0, 0, 0};
		gyroG_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
		TG_speed = (struct Gradual){0, 0, 0};
		gyroT_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
		break;
	}
	case is_sp:
	{
		break;
	}
	}
	PIDMode = target_mode;
}

/**
 * @brief: 
 * @return {*}
 */
void get_motor_speed()
{
	Speed[0] = (short)TIM1->CNT;
	TIM1->CNT = 0;

	// 左后轮
	Speed[1] = (short)TIM2->CNT;
	TIM2->CNT = 0;

	// 右前轮
	Speed[2] = (short)TIM3->CNT;
	TIM3->CNT = 0;

	// 右后轮
	Speed[3] = (short)TIM5->CNT;
	TIM5->CNT = 0;

	if(ScanMode == is_Front)
	{
		dirct[0] = dirct[3] = 1;
		dirct[2] = dirct[1] = -1;
		motor_L0.measure = (float)Speed[0] * dirct[0]*measure_k;
		motor_L1.measure = (float)Speed[1] * dirct[1]*measure_k; 
		motor_R0.measure = (float)Speed[2] * dirct[2];
		motor_R1.measure = (float)Speed[3] * dirct[3];
	}
	else if(ScanMode == is_Back)
	{
		dirct[0] = dirct[3] = -1;
		dirct[2] = dirct[1] = 1;
		motor_L0.measure = (float)Speed[3] * dirct[3];
		motor_L1.measure = (float)Speed[2] * dirct[2]; 
		motor_R0.measure = (float)Speed[1] * dirct[1]*measure_k;
		motor_R1.measure = (float)Speed[0] * dirct[0]*measure_k;
	}
	
}
