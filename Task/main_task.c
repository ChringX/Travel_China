/*
 * @File: main_task.c
 * @Description: 
 * @Version: 1.0.0
 * @Author: 
 * @Date: 2023-09-13 20:33:36
 * @LastEditTime: 2023-09-15 16:27:25
 */
#include "main_task.h"
#include "rudder_control.h"
#include "uart.h"
#include "imu_task.h"
#include "uart.h"
#include "turn.h"
#include "map.h"
#include "barrier.h"
#include "bsp_buzzer.h"
#include "bsp_linefollower.h"
#include "scaner.h"
#include "speed_ctrl.h"
#include "encoder.h"
#include "barrier.h"
#include "motor_task.h"
#include "openmv.h"
#include "math.h"
#include "barrier.h"

/**
 * @brief: 
 * @param {void} *pvParameters
 * @return {*}
 */
void main_task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();   //获取系统节拍
	
	zhunbei();
	
	while(1)
	{
		if(map.routetime == 1)
		{
			map.routetime = 2;
			get_newroute();
			float mpuZ_reset_val;
			for (uint8_t i = 0; i<10; i++)
			{
				delay_ms(20);
				mpuZ_reset_val += imu.yaw;  
			}
			mpuZ_reset_val /= 10;   
			mpuZreset(mpuZ_reset_val, nodesr.nowNode.angle);//把此时角度变为此结点角度
			zhunbei(); 
		}
		
		Cross();
		
		if(map.routetime==3)
		{
			CarBrake();
			while(1)
			{
				vTaskDelay(2);
			}
		}
	
		vTaskDelayUntil(&xLastWakeTime, (5/portTICK_RATE_MS));//绝对休眠5ms // INCLUDE_vTaskDelayUntil 1
	}
}
