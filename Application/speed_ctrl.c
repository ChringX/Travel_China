#include "speed_ctrl.h"
#include "motor_task.h"
#include "pid.h"
#include "motor.h"
#include "bsp_buzzer.h"



volatile struct Motors motor_all = {
	.Lspeed = 0,
	.Rspeed = 0,
	.encoder_avg = 0,
	.GyroG_speedMax = 100, 	// 自平衡左右偏差最大值10000
	.GyroT_speedMax = 25,  	// 自转最大速度34//--->5760 //35
	.Cincrement = 0.25,	   	// 循迹加速度
	.CDOWNincrement = 0.6,	//循迹减速
	.Gincrement = 0.5,	   	// 非循迹加速度
	.GDOWNincrement=0.5,
	.is_UP = false,
	.is_DOWM = false,
};

struct Gradual TC_speed = {0, 0, 0}, TG_speed = {0, 0, 0}, TP_speed = {0, 0, 0}, TCO_speed = {0, 0, 0};

/**
 * @brief:
 * @return {*}
 */
void CarBrake(void)
{
	// 闭环
	// pid_mode_switch(is_No);
	// motor_all.Lspeed = motor_all.Rspeed = 0;

	// 开环
	pid_mode_switch(is_Free);
	motor_set_pwm(1, 0);
	motor_set_pwm(2, 0);
	motor_set_pwm(3, 0);
	motor_set_pwm(4, 0);

	motor_pid_clear();
	TC_speed = (struct Gradual){0, 10, 0};
}

/**
 * @brief: 以一次函数缓慢加速或者缓慢停止
 * @param {Gradual} *gradual
 * @param {float} target
 * @param {float} increment
 * @return {*}
 */
void gradual_cal(struct Gradual *gradual, float target, float increment1, float increment2)
{
	uint8_t direction = 0;

	if (target - gradual->Now < 0) // 减速
		direction = 0;
	else
		direction = 1;
	
	if (gradual->Now != target)
	{
		if (direction)
			gradual->Now += increment1;
		else
			gradual->Now -= increment2;
	}
	else
	{
		return;
	}

	if (direction == 1) // 加速
	{
		if (gradual->Now > target)
		{
			gradual->Now = target;
		}
	}
	else if (direction == 0)
	{
		if (gradual->Now < target) // 速度小于目标速度
		{
			gradual->Now = target;
		}
	}
}

/*卡死停车*/
void CarBrake_Stop(void)
{
	buzzer_on();
	while(1)
	{
		CarBrake();
		vTaskDelay(2);
	}
}
