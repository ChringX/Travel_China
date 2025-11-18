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

void Arrived_Stage(void)
{
	Rudder_control(150, 2); // 450左手放下  150举起
	vTaskDelay(50);
	Rudder_control(450, 2); // 450左手放下  150举起
	Rudder_control(400, 4); // 130右手放下  400举起
	vTaskDelay(50);
	Rudder_control(130, 4); // 130右手放下  400举起
}

