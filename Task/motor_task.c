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
#include "K210.h"



TaskHandle_t motor_handler;
int dirct[4] = {-1, 1, -1, -1};
volatile uint8_t PIDMode;
uint8_t line_gyro_switch = 0;
uint8_t Nosmall = 1;
int MOTOR_PWM_MAX = 9800;



/*电机任务*/
void motor_task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount(); 	// 获取系统节拍
	static uint8_t mouse = 0;			 	// 小灯鼠
	while (1)
	{
		/*获取循迹值*/
		if (PIDMode == is_Line)
			getline_error();

		/*获取电机速度与路程*/
		get_motor_speed();
		motor_all.encoder_avg = (motor_L0.measure + motor_L1.measure + motor_R0.measure + motor_R1.measure) / 4;
		motor_all.Distance += ((motor_all.encoder_avg * 10.4f * PI)/5720.0f)/0.362f;
	
		/*是否开启红外*/
		if(infrare_open)
			get_Infrared();
	
		/*模式切换处理*/
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
			/*循迹模式*/
			if (PIDMode == is_Line)
			{
				if(nodesr.nowNode.speed >= SPEED3 && fabsf(Scaner.error) < 1)
					Scaner.error /= 4;
				gradual_cal(&TC_speed, motor_all.Cspeed, motor_all.Cincrement,motor_all.CDOWNincrement);
				Go_Line(TC_speed.Now);
			}
			else
				motor_all.Cspeed = 0;
	
			/*转弯模式*/
			if (PIDMode == is_Turn)
			{
				if (Turn360_Flag)
					Turn360Step();
				else if (nodesr.nowNode.function == UpStage || nodesr.nowNode.function == BSoutPole || nodesr.nowNode.function == BHM)
				{
					if (Stage_turn_Angle(angle.AngleT))
						gyroT_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0};
				}
				else if (Turn_Angle(angle.AngleT))
					gyroT_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0};
			}
	
			/*自平衡模式*/
			if (PIDMode == is_Gyro)
			{
				gradual_cal(&TG_speed, motor_all.Gspeed, motor_all.Gincrement,motor_all.GDOWNincrement);
				runWithAngle(angle.AngleG, TG_speed.Now);
			}
			else
				motor_all.Gspeed = 0;
		}
	
		/*灯鼠*/
		mouse++;
		if (mouse > 100)
		{
			mouse = 0;
			LED_C1_Toggle();
		}
	
		/*赋目标速度*/
		if(DownLiuShui)
		{
			motor_L0.target = motor_all.Lspeed * LiuShuiRate;
			motor_L1.target = motor_all.Lspeed;
			motor_R0.target = motor_all.Rspeed * LiuShuiRate;
			motor_R1.target = motor_all.Rspeed;
		}
		else if(WavePlateLeft_Flag)
		{
			motor_L0.target = motor_L1.target = motor_all.Lspeed * 1.2f;
			motor_R0.target = motor_R1.target = motor_all.Rspeed;
		}
		else if(WavePlateRight_Flag)
		{
			motor_L0.target = motor_L1.target = motor_all.Lspeed;
			motor_R0.target = motor_R1.target = motor_all.Rspeed * 1.2f;
		}
		else
		{
			motor_L0.target = motor_L1.target = motor_all.Lspeed;
			motor_R0.target = motor_R1.target = motor_all.Rspeed;
		}
		
		/*模式判断*/
		if (PIDMode != is_Free) // 判断是否要自控
		{
			/*PID计算*/
			incremental_PID(&motor_L0, &motor_pid_paramL0);
			incremental_PID(&motor_L1, &motor_pid_paramL1);
			incremental_PID(&motor_R0, &motor_pid_paramR0);
			incremental_PID(&motor_R1, &motor_pid_paramR1);

			motor_set_pwm(1,(int32_t)motor_L0.output);
			motor_set_pwm(2,(int32_t)motor_L1.output);
			motor_set_pwm(3,(int32_t)motor_R0.output);
			motor_set_pwm(4,(int32_t)motor_R1.output);
		}

		// /*陀螺仪模式*/ printf("Gyro:%.2f LSP:%.2f RSP:%.2f L0:%.2f L1:%.2f R0:%.2f R1:%.2f\r\n", imu.yaw,motor_all.Lspeed,motor_all.Rspeed,motor_L0.output,motor_L1.output,motor_R0.output,motor_R1.output);
		// /*电机环PID*/ printf("LSP:%.2f RSP:%.2f L0:%.2f L1:%.2f R0:%.2f R1:%.2f\r\n", motor_all.Lspeed,motor_all.Rspeed,motor_L0.output,motor_L1.output,motor_R0.output,motor_R1.output);
		// /*电机环目标值*/ printf("L0tar:%.2f\tL1tar:%.2f\tR0tar:%.2f\tR1tar:%.2f\r\n", motor_L0.target, motor_L1.target, motor_R0.target, motor_R1.target);
		// /*循迹值*/ printf_byte(Scaner.detail);
		// /*陀螺仪读数*/ printf("yaw:%.2f\troll:%.2f\tpitch:%.2f\tbasic:%.2f\r\n", imu.yaw, imu.roll, imu.pitch, basic_p);
		// /*当前目标节点*/ printf("%d\r\n",nodesr.nowNode.nodenum);
		// /*三门大炮*/ printf("前%d 左%d 右%d\r\n", Infrared_ahead, infrared.head_left, infrared.head_right);
		// /*各轮子测量值*/ printf("L0:%.1f,L1:%.1f,R0:%.1f,R1:%.1f\r\n", motor_L0.measure, motor_L1.measure, motor_R0.measure, motor_R1.measure);
		// /*打印识别数字*/ printf("%d\r\n", Clue_Num);

		vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS)); // 绝对休眠5ms // INCLUDE_vTaskDelayUntil 1
	}
}

/*创建电机任务*/
void motor_task_create(void)
{
	xTaskCreate((TaskFunction_t)motor_task,		  // 任务函数
				(const char *)"motor_task",		  // 任务名字
				(uint32_t)motor_size,			  // 任务堆栈大小
				(void *)NULL,					  // 传递给任务参数的指针参数
				(UBaseType_t)motor_task_priority, // 任务的优先级
				(TaskHandle_t *)&motor_handler);  // 任务句柄
}

/*模式转换*/
void pid_mode_switch(uint8_t target_mode)
{
	switch (target_mode)
	{
		case is_Turn:
		{
			MOTOR_PWM_MAX = 5000;
			line_pid_obj = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
			gyroG_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
			gyroT_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
			TC_speed = (struct Gradual){0, 0, 0};
			TG_speed = (struct Gradual){0, 0, 0};
			break;
		}

		case is_Line:
		{
			MOTOR_PWM_MAX = 9800;		//9800
			if(PIDMode == is_Gyro) 				//自平衡->循线
				line_gyro_switch = 1;
			else
				gyroT_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
			break;
		}

		case is_Gyro:
		{
			MOTOR_PWM_MAX = 9800;
			if (PIDMode == is_Line)				//循线->自平衡
				line_gyro_switch = 2;
			else
				gyroT_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
			break;
		}

		case is_Free:
		{
			MOTOR_PWM_MAX = 9800;
			break;
		}

		case is_No:
		{
			MOTOR_PWM_MAX = 9800;
			line_pid_obj = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
			gyroT_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
			gyroG_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
			TG_speed = (struct Gradual){0, 0, 0};
			TC_speed = (struct Gradual){0, 0, 0};
			break;
		}
	}

	PIDMode = target_mode;
}

/*获取编码器测量值*/
void get_motor_speed()
{
	// 左前轮
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

	dirct[0] = dirct[1] = 1;
	dirct[2] = dirct[3] = -1;
	motor_L0.measure = (float)Speed[0] * dirct[0];
	motor_L1.measure = (float)Speed[1] * dirct[1];
	motor_R0.measure = (float)Speed[2] * dirct[2];
	motor_R1.measure = (float)Speed[3] * dirct[3];
}
