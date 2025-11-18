/*
 * @File: motion.c
 * @Description:
 * @Version: 1.0.0
 * @Author:
 * @Date: 2023-09-13 20:33:34
 * @LastEditTime: 2023-09-15 16:07:12
 */
#include "iic.h"
#include "turn.h"
#include "bsp_led.h"
#include "rudder_control.h"
#include "motion.h"

/*机器人动作*/
void Arrived_Stage(void)
{
	Robot_Work(LARM, UP); // 450左手放下  150举起
	vTaskDelay(100);
	Robot_Work(RARM, UP); // 130右手放下  400举起
	vTaskDelay(200);
	Robot_Work(LARM, DOWN); // 450左手放下  150举起
	vTaskDelay(100);
	Robot_Work(RARM, DOWN); // 130右手放下  400举起
}

