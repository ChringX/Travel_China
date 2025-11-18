/*
 * @File: barrier.c
 * @Description:
 * @Version: 1.0.0
 * @Author:
 * @Date: 2023-09-13 20:33:33
 * @LastEditTime: 2023-09-15 16:13:06
 */
#include "barrier.h"
#include "sys.h"
#include "delay.h"
#include "speed_ctrl.h"
#include "motor.h"
#include "pid.h"
#include "imu_task.h"
#include "scaner.h"
#include "turn.h"
#include "map.h"
#include "motor_task.h"
#include "pid.h"
#include "math.h"
#include "bsp_buzzer.h"
#include "bsp_led.h"
#include "stdio.h"
#include "motor_task.h"
#include "QR.h"
#include "motion.h"
#include "bsp_linefollower.h"
#include "openmv.h"
#include "Rudder_control.h"
#include "encoder.h"
#include "filter.h"
#include "K210.h"
#include "string.h"

#define BACK_SPEED -7 //-15
#define View_BACK_SPEED -20
#define IMPACT_SPEED 13

extern int BW_num[];
uint8_t color_flag[5] = {0,0,0,0,0};
uint8_t stage1 = 0;
uint8_t treasure = 0;
uint8_t value; // openmv接口
uint8_t DownLiuShui = 0;
uint8_t special_arrive = 0;
uint8_t R_Fight = 0;
uint8_t L_Fight = 0;
// 防抖动系列函数
// 1._shake在平台识别过程中，防止因一些意外因素导致车提前停止,同时完成撞板的目的
// 2._Rshake用于判断其是否经过波浪板
// 3._pshake识别处于下坡状态
void Anti_shake(int Uneed_time)
{
	uint32_t i = 0; // 循环变量设置成unit32_t避免for循环最后的奇怪指令
	for (i = 0; i < Uneed_time; i++)
	{

		while (Infrared_ahead == 1)
		{
			vTaskDelay(1);
		}
	}
}

/**
 * @brief:
 * @param {int} Uneed_time
 * @return {*}
 */
int AI_shake(int Uneed_time)
{
	//	uint32_t i = 0, cnt = 0;  //循环变量设置成unit32_t避免for循环最后的奇怪指令
	//	for(i = 0; i < Uneed_time ; i++)
	//	{
	//		HAL_Delay(1);
	//		if(AI == 1)
	//		{
	//			cnt++;
	//		}
	//	}
	//	if(cnt >= Uneed_time*0.5)  return 1;
	return 0;
}

// void Anti_encoder(int Uneed_time)
//{
//	uint32_t i = 0;  //循环变量设置成unit32_t避免for循环最后的奇怪指令
//	for(i = 0; i < Uneed_time ; i++)
//	{
//		HAL_Delay(1);
//		while(read_encoder(2) == 0);
//	}
// }

// void Anti_Rshake()
//{
//	uint32_t i = 0;
//	uint32_t cnt = 0;
//	while(i < 148)
//	{
//		for(i = 0; i < 150 ; i++)
//		{
//			HAL_Delay(1);
//			if(fabs(imu.roll) > 2) cnt++;
//			if(cnt == 30)
//			{
//				i = 0;
//				cnt = 0;
//				break;
//			}
//			//while(fabs(imu.roll) > 2);
//		}
//	}
// }

/**
 * @brief:
 * @return {*}
 */
int Anti_Pshake(void)
{
	int sum = 0;
	for (uint32_t i = 0; i < 5; i++)
	{
		HAL_Delay(1);
		sum += imu.pitch;
	}
	if (sum / 5 > 3)
		return 1;
	else
		return 0;
}

/**
 * @brief:
 * @return {*}
 */
void Stage(void) // flag==1时取绝度角度，flag==0时取相对角度
{
	int break_times = 0;
	static uint8_t first2P1 = 0;
	stage1 = 1;
	struct PID_param origin_param = gyroT_pid_param;
	struct PID_param origin_param1 = gyroG_pid_param;
	float num = 0;
	nodesr.nowNode.flag = 0;

	Rudder_control(170, 0); // 人站起来

	gyroT_pid_param.kp = 0.5;
	gyroT_pid_param.ki = 0.004;
	gyroT_pid_param.kd = 0;

	gyroG_pid_param.kp = 1;
	gyroG_pid_param.ki = 0;
	gyroG_pid_param.kd = 0.5;

	encoder_clear();
	num = motor_all.Distance;
	while (Scaner.ledNum < 4 || Scaner.lineNum < 2) // 
	{
		vTaskDelay(2);
	}
	angle.AngleG = getAngleZ();
	motor_all.Gspeed = GoStage_Speed;
	pid_mode_switch(is_Gyro);

	if (nodesr.nowNode.nodenum == P2)
	{
		Stage_P2();
		return;
	}
	while (imu.pitch < Up_pitch - 3)
	{
		vTaskDelay(5);
	} // 刚上
	while (imu.pitch > After_up)
	{
		vTaskDelay(5);
	} // 上完桥

	angle.AngleG = getAngleZ();
	motor_all.Gspeed = GoStage_Speed;
	pid_mode_switch(is_Gyro);
	encoder_clear();
	
	if(ScanMode == is_Front)
	{
		while (Infrared_ahead == 0)
		{
			vTaskDelay(5);
		} // 撞挡板
	}
	else if(ScanMode == is_Back)
	{
		while (Infrared_back == 0)
		{
			vTaskDelay(5);
		} // 撞挡板
	}
	
	motor_all.Gspeed = IMPACT_SPEED;
	num = motor_all.Distance;
	while (fabsf(motor_all.Distance - num) < 19 * 60) // 前进一段距离//23
	{
		vTaskDelay(5);
	}

	mpuZreset(imu.yaw, nodesr.nowNode.angle); // 陀螺仪校正
	encoder_clear();
	
	CarBrake();
	vTaskDelay(250);

	if(map.routetime == 0)
	{
		if (nodesr.nowNode.nodenum == P1 && first2P1 == 0)
		{
			first2P1++;
			while (Clue_Stage[0] == 0 || Clue_Stage[1] == 0 || Clue_Stage[2] == 0)
			{
				vTaskDelay(2);
				break_times++;
				if (break_times >= 500)
				{
					num = motor_all.Distance;
					motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;
					while (fabsf(motor_all.Distance - num) < 60) // 360//60
					{
						vTaskDelay(2);
					}
					CarBrake();
					encoder_clear();
					break_times = 0;
				}
			} // 等待宝藏
			Connect_Route();
		}
		else if((Clue_Stage[1] == 5 && nodesr.nowNode.nodenum == P6)
			 || (Clue_Stage[1] == 6 && nodesr.nowNode.nodenum == P5))
		{
			WaitForK210();//得到num1或者宝藏的位置
		}
	}
	
	num = motor_all.Distance; // 后退一段距离
	pid_mode_switch(is_No);
	motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;
	while (fabsf(motor_all.Distance - num) < 360) // 300
	{
		vTaskDelay(2);
	}
	encoder_clear();

	CarBrake();
	vTaskDelay(250);
	
	Arrived_Stage();
	
	switch (nodesr.nowNode.nodenum)
	{
		case P1: send_play_specified_command(28); break;
		case P3: send_play_specified_command(27); break;
		case P4: send_play_specified_command(26); break;
		case P5: send_play_specified_command(24); break;
		case P6: send_play_specified_command(25); break;
		case P7: send_play_specified_command(36); break;
		case P8: send_play_specified_command(34); break;
		default: break;
	}

	Turn_Angle_Relative_Open(179);
	while (fabsf(need2turn(angle.AngleT, getAngleZ())) > 4) // 7
	{
		vTaskDelay(2);
	}

	if (treasure == nodesr.nowNode.nodenum) // 发现宝藏的动作
	{
		// 左边 450放下 150举起
		// 右边 130放下 400举起
		motor_pid_clear();
		Rudder_control(150, 2);
		Rudder_control(400, 4);
		send_play_specified_command(32);
		Turn_Angle360();
		Rudder_control(450, 2);
		Rudder_control(130, 4);
	}
	if((Clue_Stage[1] == 5 && nodesr.nowNode.nodenum == P6)
	|| (Clue_Stage[1] == 6 && nodesr.nowNode.nodenum == P5))
	{
		switch (Clue_Num[0])
		{
			case 0: send_play_specified_command(16); break;
			case 1: send_play_specified_command(14); break;
			case 2: send_play_specified_command(11); break;
			case 3: send_play_specified_command(9); break;
			case 4: send_play_specified_command(8); break;
			case 5: send_play_specified_command(22); break;
			case 6: send_play_specified_command(20); break;
			default: break;
		}
	}
	motor_pid_clear();
	
	DownLiuShui = 1;
	motor_all.Cspeed = Rubbish_Speed;
	pid_mode_switch(is_Line);
	while (imu.pitch > Down_pitch)
	{
		vTaskDelay(2);
	}
	num = motor_all.Distance;
	while (imu.pitch < After_down)
	{
		vTaskDelay(2);
		if (fabsf(motor_all.Distance - num) < 12 * 60)
			break;
	}
	DownLiuShui = 0;

	gyroT_pid_param = origin_param;
	gyroG_pid_param = origin_param1;
	Rudder_control(270,0);// 270 人躺下		 // 人躺下
	nodesr.nowNode.function = 0; // 清除障碍标志
	nodesr.flag |= 0x04;		 // 到达路口
}

/**
 * @brief: 平台二的动作
 * @return {*}
 */
void Stage_P2() // flag==1时取绝度角度，flag==0时取相对角度
{
	static uint8_t Backtimes = 0;
	while (imu.pitch < basic_p + 12)
	{
		vTaskDelay(2);
	}
	while (imu.pitch > After_up)
	{
		vTaskDelay(2);
	}
	encoder_clear();
	float num = motor_all.Distance;
	while (fabsf(motor_all.Distance - num) < 13 * 60) // 7
	{
		vTaskDelay(2);
	}
	CarBrake();
	vTaskDelay(2);
	if (Backtimes == 1)
	{
		while (1);
	}
	Backtimes++;
	Change_MODE();
	motor_all.Cspeed = 0;
	motor_pid_clear();
	nodesr.nowNode.function = 0; // 清除障碍标志
	nodesr.flag |= 0x04;		 // 到达路口
}

/**
 * @brief: 过长桥
 * @param {float} step
 * @param {float} speed
 * @return {*}
 */
void Barrier_Bridge(float step, float speed)
{
	float num = 0;
	float now_angle = 0;
	struct PID_param origin_param1 = gyroG_pid_param;
	struct PID_param origin_param2 = line_pid_param;

	gyroG_pid_param.kp = 1.1; //2  //1.5  //1.2
	gyroG_pid_param.ki = 0.004;
	gyroG_pid_param.kd = 0;//1
	
	line_pid_param.kp = 30;
	line_pid_param.ki = 0;
	line_pid_param.kd = 5;
	
	motor_all.Cspeed = Rubbish_Speed;
	pid_mode_switch(is_Line);
	encoder_clear();
	num = motor_all.Distance;
	while (fabsf(motor_all.Distance - num) < 10000)//1500
	{
		if (Scaner.ledNum >= 4 || Scaner.lineNum >= 2)
			break;
		vTaskDelay(2);
	}
	mpuZreset(imu.yaw, nodesr.nowNode.angle);
	encoder_clear();

	angle.AngleG = getAngleZ();	  // 自平衡走的角度
	motor_all.Gspeed = Rubbish_Speed; // 自平衡速度
	pid_mode_switch(is_Gyro);

	while (imu.pitch < Up_pitch) // 还在平地,出循环就是上桥中
	{
		vTaskDelay(2);
	}
	now_angle = angle.AngleG; 

	while (imu.pitch > After_up)
	{
		vTaskDelay(2);
		getline_error();
		if(ScanMode == is_Front)
		{
			if ((infrared.head_left == 0 && infrared.head_right == 1) || (Scaner.detail & 0X0003))
			{
					angle.AngleG = now_angle + 2.5f; // 4
			}
			else if ((infrared.head_left == 1 && infrared.head_right == 0) || (Scaner.detail & 0XC000))
			{
					angle.AngleG = now_angle - 2.5f; // 4
			}
		}
		else if(ScanMode == is_Back)
		{
			if ((infrared.tail_left == 0 && infrared.tail_right == 1) || (Scaner.detail & 0X0003))
			{
					angle.AngleG = now_angle + 2.5f; // 4
			}
			else if ((infrared.tail_left == 1 && infrared.tail_right == 0) || (Scaner.detail & 0XC000))
			{
					angle.AngleG = now_angle - 2.5f; // 4
			}
		}
	}
	num = motor_all.Distance;
	infrare_open = 1;
	buzzer_on();
	while (fabsf(motor_all.Distance - num) < 300)
	{
		vTaskDelay(2);
		getline_error();
		if(ScanMode == is_Front)
		{
			if ((infrared.head_left == 0 && infrared.head_right == 1) || (Scaner.detail & 0X0003))
			{
					angle.AngleG = now_angle + 2.5f; // 2
			}
			else if ((infrared.head_left == 1 && infrared.head_right == 0) || (Scaner.detail & 0XC000))
			{
					angle.AngleG = now_angle - 2.5f; // 2
			}
		}
		else if(ScanMode == is_Back)
		{
			if ((infrared.tail_left == 0 && infrared.tail_right == 1) || (Scaner.detail & 0X0003))
			{
					angle.AngleG = now_angle + 2.5f; // 2
			}
			else if ((infrared.tail_left == 1 && infrared.tail_right == 0) || (Scaner.detail & 0XC000))
			{
					angle.AngleG = now_angle - 2.5f; // 2
			}
		}
	}
	encoder_clear();
	num = motor_all.Distance;
	motor_all.Gspeed = Bridge_Speed;
	while (imu.pitch > basic_p - 5)
	{
		vTaskDelay(2);
		getline_error();
		if(ScanMode == is_Front)
		{
			if ((infrared.head_left == 0 && infrared.head_right == 1) || (Scaner.detail & 0X0003)) //000F
			{
					angle.AngleG = now_angle + 2.5f; // 2
			}
			else if ((infrared.head_left == 1 && infrared.head_right == 0) || (Scaner.detail & 0XC000)) //F000
			{
					angle.AngleG = now_angle - 2.5f; // 2
			}
		}
		else if(ScanMode == is_Back)
		{
			if ((infrared.tail_left == 0 && infrared.tail_right == 1) || (Scaner.detail & 0X0007))
			{
					angle.AngleG = now_angle + 2.5f; // 2
			}
			else if ((infrared.tail_left == 1 && infrared.tail_right == 0) || (Scaner.detail & 0XE000))
			{
					angle.AngleG = now_angle - 2.5f; // 2
			}
		}
		if (fabsf(motor_all.Distance - num) > 5500) // 3900 
		{
			motor_all.Gspeed = Rubbish_Speed;
		}
		if (fabsf(motor_all.Distance - num) > 7000) 
		{
			break;
		}
	}
	buzzer_off();
	while(imu.pitch > -15)
	{
		vTaskDelay(2);
	}
	
	gyroG_pid_param = origin_param1;
	
	DownLiuShui = 1;
	while(imu.pitch <= -15)
	{
		vTaskDelay(2);
	}
	DownLiuShui = 0;
	
	origin_param1 = gyroG_pid_param;
	gyroG_pid_param.kp = 0.5;
	gyroG_pid_param.ki = 0;
	gyroG_pid_param.kd = 0.5;
	
//	angle.AngleG = getAngleZ();
	while ((imu.pitch < (After_down + 1) && (infrared.head_left == 0 || infrared.head_right == 0))
		|| (imu.pitch < (After_down + 1) && (infrared.tail_left == 0 || infrared.tail_right == 0)))
	{
		vTaskDelay(2);
	} // 出循环下完在平地
	infrare_open = 0;
	motor_all.Cspeed = Rubbish_Speed;
	pid_mode_switch(is_Line);
	gyroG_pid_param = origin_param1;
	line_pid_param = origin_param2;
	// 路程记录清零
	motor_all.Distance = 0;
	motor_all.encoder_avg = 0;
	nodesr.nowNode.function = 0;
	nodesr.flag |= 0X04; // 到达路口
}

/**
 * @brief: 过楼梯
 * @param {uint8_t} order
 * @return {*}
 */
void Barrier_Hill(uint8_t order) // 楼梯数量
{
	struct PID_param origin_param1 = line_pid_param;
	float num = 0;
	motor_all.Cspeed = GoStage_Speed;
	
	if((Scaner.detail&0X180) == 0X180)
		mpuZreset(imu.yaw,nodesr.nowNode.angle);
	
	line_pid_param.kp = 20;//22 //27  //20
	line_pid_param.ki = 0.004;
	line_pid_param.kd = 0;
	
	buzzer_on();
	while (imu.pitch < basic_p + 8)
	{
		BHillChange();
		vTaskDelay(2);
	}
	while (imu.pitch > basic_p - 6)
	{
		BHillChange();
		vTaskDelay(2);
	} // 开始下楼梯
	BHillChange();
	num = motor_all.Distance;
	while (imu.pitch < basic_p - 5)
	{
		BHillChange();
		vTaskDelay(2);
		if (fabsf(motor_all.Distance - num) > 630)
			break;
	} // 下完楼梯
	buzzer_off();
	pid_mode_switch(is_Line);
	motor_all.Cspeed = Rubbish_Speed;
	
	encoder_clear();
	nodesr.nowNode.function = 0; // 清除障碍标志
	line_pid_param = origin_param1;
	nodesr.flag |= 0x04;		 // 到达路口
}

/**
 * @brief:
 * @return {*}
 */
void Sword_Mountain(void)
{
	float num;
	struct PID_param origin_param = line_pid_param;
	struct PID_param origin_param1 = gyroG_pid_param;
	motor_all.Cspeed = Stop_T_Speed;

	line_pid_param.kp = 35; // 25
	line_pid_param.ki = 0;
	line_pid_param.kd = 4; // 4

	gyroG_pid_param.kp = 1;
	gyroG_pid_param.ki = 0;
	gyroG_pid_param.kd = 0.5;
	
	encoder_clear();
	num = motor_all.Distance;
	while (fabsf(motor_all.Distance - num) < 10 * 60) // 强矫正循迹位置 //20
	{
		vTaskDelay(2);
	}
	mpuZreset(imu.yaw, nodesr.nowNode.angle); // 陀螺仪校正

	getline_error();
	while (Scaner.ledNum <= 4)
	{
		getline_error();
		vTaskDelay(2);
	}

	angle.AngleG = getAngleZ();
	buzzer_on();

	gyroG_pid_param.kp = 12.4; // 拉大刀山的自平衡kp
	motor_all.Gspeed = Stop_T_Speed;
	pid_mode_switch(is_Gyro);
	num = motor_all.Distance;
	while (imu.pitch < After_up) // 出循环上刀山
	{
		vTaskDelay(2);
		if (fabsf(motor_all.Distance - num) > 30 * 60)
			break;
	}

	while (imu.pitch > After_down) // 出循环下刀山
	{
		vTaskDelay(2);
	}

	line_pid_param = origin_param;
	gyroG_pid_param = origin_param1;
	buzzer_off();

	nodesr.nowNode.function = 0; // 清除障碍标志
	nodesr.flag |= 0x04;		 // 到达路口
}

/**
 * @brief: 上珠峰，包含下珠峰
 * @param {float} speed
 * @return {*}
 */
void Barrier_HighMountain(float speed)
{
	int UpSpeedTime = 0;
	float num = 0;
	struct PID_param origin_param = line_pid_param;
	struct PID_param origin_param1 = gyroG_pid_param;
	float origin_turnM = motor_all.GyroT_speedMax;

	motor_all.GyroT_speedMax = 25;
	
	motor_all.Cspeed = speed;
	
	gyroG_pid_param.kp = 0.5;
	gyroG_pid_param.ki = 0;
	gyroG_pid_param.kd = 0.5;

	line_pid_param.kp = 26.5;  //15
	line_pid_param.ki = 0; 
	line_pid_param.kd = 1.5; //1.5

	if((Scaner.detail & 0X180) == 0X180)
	{
		mpuZreset(imu.yaw,nodesr.nowNode.angle);
	}
	
	while (Scaner.ledNum < 4 || Scaner.lineNum < 2) // 前进一段距离//760
	{
		vTaskDelay(2);
	}
	
	// 上桥陀螺仪
	angle.AngleG = nodesr.nowNode.angle;
	motor_all.Gspeed = speed;
	pid_mode_switch(is_Gyro);
	encoder_clear();
	while(imu.pitch < Up_pitch+10)
	{
		vTaskDelay(2);
	} // 刚上山
//	while(fabsf(motor_all.Distance - num) < 15 * 60) // 上坡走循迹  //70
//	{
//		vTaskDelay(2);
//	}
	buzzer_on();
	
	motor_all.Cspeed = speed;
	pid_mode_switch(is_Line);
	encoder_clear();
	num = motor_all.Distance;
	while(fabsf(motor_all.Distance - num) < 80 * 60) // 上坡走循迹  //70
	{
		vTaskDelay(2);
	}
//	mpuZreset(imu.yaw,nodesr.nowNode.angle);
	buzzer_off();
	// 第一个平地+上桥自平衡

	motor_all.Gspeed = speed;
	angle.AngleG = getAngleZ();
	pid_mode_switch(is_Gyro);
	while (imu.pitch > After_up)
	{
		vTaskDelay(2);
	} // 到平地
	motor_all.Gspeed = speed;
	while (imu.pitch < Up_pitch)
	{
		vTaskDelay(2);
	} // 第二个坡

	Rudder_control(170, 0); // 320躺下 170站起来

	// 上桥中循迹过
	motor_all.Cspeed = speed;
	pid_mode_switch(is_Line);
	buzzer_on();
	while(UpSpeedTime < 1000)
	{
		Cross_getline();
		if(Cross_Scaner.detail & 0X180)
			UpSpeedTime += 5;
		else
			UpSpeedTime +=1;
		vTaskDelay(2);
	}
	UpSpeedTime = 0;
	buzzer_off();
	motor_all.Cspeed = Mount_Speed;

	encoder_clear();
	num = motor_all.Distance;
	uint8_t DownTime = 0;
	while (fabsf(motor_all.Distance - num) < 6000)
	{
		vTaskDelay(2);
		if(DownTime == 0 && fabsf(motor_all.Distance - num) > 5000)
		{
			DownTime++;
			motor_all.Cspeed = speed;
		}
		if (Scaner.ledNum >= 7)
			break;
	}
	buzzer_off();

	// 第二次上平台ing 自平衡
	motor_all.Gspeed = IMPACT_SPEED;
	angle.AngleG = getAngleZ();
	pid_mode_switch(is_Gyro);

	if(ScanMode == is_Front)
	{
		while (Infrared_ahead == 0)
		{
			vTaskDelay(5);
		} // 撞挡板
	}
	else if(ScanMode == is_Back)
	{
		while (Infrared_back == 0)
		{
			vTaskDelay(5);
		} // 撞挡板
	}

	encoder_clear();
	num = motor_all.Distance;
	while (fabsf(motor_all.Distance - num) < 23*60) // 装挡板前进一段距离///730
	{
		vTaskDelay(5);
	}

	CarBrake();
	vTaskDelay(250);
	mpuZreset(imu.yaw, nodesr.nowNode.angle); // 陀螺仪校正
	
	if(Clue_Stage[2] == 8 && nodesr.nowNode.nodenum == P7)
	{
		WaitForK210();
	}

	Arrived_Stage();

	encoder_clear();
	pid_mode_switch(is_No);
	motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;
	num = motor_all.Distance;
	while (fabsf(num - motor_all.Distance) < 360) // 60
	{
		vTaskDelay(2);
	}

	CarBrake();
	vTaskDelay(100);
	
	send_play_specified_command(36);
	
	Turn_Angle_Relative_Open(179);				 // 转180
	while (fabs(angle.AngleT - getAngleZ()) > 5) // 判断误差
	{
		vTaskDelay(2);
	}

	if((Clue_Stage[2] == 8 && nodesr.nowNode.nodenum == P7))
	{
		switch (Clue_Num[1])
		{
			case 0: send_play_specified_command(21); break;
			case 1: send_play_specified_command(19); break;
			case 2: send_play_specified_command(18); break;
			case 3: send_play_specified_command(15); break;
			case 4: send_play_specified_command(17); break;
			case 5: send_play_specified_command(12); break;
			case 6: send_play_specified_command(10); break;
			default: break;
		}
	}
	
	if(map.routetime == 0)
		ConnectFirstBack();
	
	// 掉头之后陀螺仪自平衡
	angle.AngleG = getAngleZ();
	motor_all.Gspeed = 14; // 先变模式再赋值
	pid_mode_switch(is_Gyro);
	Rudder_control(270, 0);
	Barrier_Down_HighMountain(6);
	line_pid_param = origin_param;
	gyroG_pid_param = origin_param1;
	motor_all.GyroT_speedMax = origin_turnM;
	nodesr.nowNode.function = 0; // 清除障碍标志
	nodesr.flag |= 0x04;		 // 到达路口
}

/**
 * @brief: 下珠峰，在转完180后调用
 * @param {float} speed
 * @return {*}
 */
void Barrier_Down_HighMountain(float speed)
{
	uint8_t i = 0;
	float num = 0;
	int UpSpeedTime = 0;
//	struct PID_param origin_param = line_pid_param;
	
	while (Scaner.ledNum <= 4)
	{
		getline_error();
		vTaskDelay(2);
	}
	while (Scaner.ledNum >= 4)
	{
		getline_error();
		vTaskDelay(2);
	}

	encoder_clear();
	num = motor_all.Distance;
	while(fabsf(motor_all.Distance - num) < 3 * 60)
	{
		vTaskDelay(2);
	}

//	{ // 保护机制
//		getline_error();
//		// 左边4
//		//		if(Scaner.detail&0x07 && ((imu.yaw>0&&imu.yaw<90) ||(imu.yaw<-90||imu.yaw==180||imu.yaw==0))) //0xf
//		//		{
//		//			//后退
//		//		//	send_play_specified_command(6);
//		//			Protect(-10.0f);
//		//		}
//		//		//左边3
//		//		else if(Scaner.detail&0x38 && ((imu.yaw>0&&imu.yaw<90) ||(imu.yaw<-90||imu.yaw==180||imu.yaw==0))) //0xf
//		//		{
//		//			//后退
//		//			//send_play_specified_command(2);
//		//			Protect(-6.0f);
//		//		}

//		// 右边4
//		if (Scaner.detail & 0xE000 && ((imu.yaw < 0 && imu.yaw > -90) || (imu.yaw > 90 || imu.yaw == 180 || imu.yaw == 0))) // 0xf000
//		{
//			// 后退
//			// send_play_specified_command(3);
//			Protect(10.0f);
//		}

//		// 右边3
//		//		else if(Scaner.detail&0x1C00 && ((imu.yaw<0&&imu.yaw>-90) ||(imu.yaw>90||imu.yaw==180||imu.yaw==0)))
//		//		{
//		//			//后退
//		//			//send_play_specified_command(4);
//		//			Protect(6.0f);
//		//		}
//	}

	pid_mode_switch(is_Gyro);
	angle.AngleG = getAngleZ();
	motor_all.Gspeed = Old_M_Speed;
	encoder_clear();
	num = motor_all.Distance;
	while (fabsf(motor_all.Distance - num) < 12 * 60) // 刹车之后要走到寻到线为止才能退出陀螺仪//1800
	{
		vTaskDelay(2);
		if (imu.pitch < Down_pitch)
			break;
	}

	motor_all.Cspeed = Rubbish_Speed;
	pid_mode_switch(is_Line);
	buzzer_on();
	while(UpSpeedTime < 1000)
	{
		Cross_getline();
		if(Cross_Scaner.detail & 0X180)
			UpSpeedTime += 5;
		else
			UpSpeedTime +=1;
		vTaskDelay(2);
	}
	UpSpeedTime = 0;
	buzzer_off();
	motor_all.Cspeed = DownBHM_Speed;
//	pid_mode_switch(is_Line);
	//	pid_mode_switch(is_Gyro);
	//	angle.AngleG=getAngleZ();
	//	motor_all.Gspeed=450;
	//	while(imu.pitch>Down_pitch)
	//	{
	//		vTaskDelay(2);
	//	}//下坡
	
	num = motor_all.Distance;
	while (fabsf(motor_all.Distance - num) < 100 * 60)  //83
	{
		vTaskDelay(2);
		if (Scaner.ledNum >= 10)
		{
			break;
//			i++;
//			if(i > 4)
//			{
//				i = 0;
//				break;
//			}
		}
	} // 先慢速
	angle.AngleG = getAngleZ();
	motor_all.Gspeed = Rubbish_Speed;
	pid_mode_switch(is_Gyro);
	while (imu.pitch < After_down)
	{
		vTaskDelay(2);
	} // 第一个平地
	while (imu.pitch > Down_pitch)
	{
		vTaskDelay(2);
		Cross_getline();
		if (Cross_Scaner.ledNum >= 10)
		{
			i++;
			if(i > 4)
			{
				i = 0;
				break;
			}
		}
	} // 刚下
	motor_all.Cspeed = Rubbish_Speed;
	pid_mode_switch(is_Line);
	buzzer_on();
	while(UpSpeedTime < 1000)
	{
		Cross_getline();
		if(Cross_Scaner.detail & 0X180)
			UpSpeedTime += 5;
		else
			UpSpeedTime +=1;
		vTaskDelay(2);
	}
	UpSpeedTime = 0;
	buzzer_off();
	motor_all.Cspeed = DownBHM_Speed;
	pid_mode_switch(is_Line);
	
	// 下坡直线途中用循迹
	while (imu.pitch < After_down)
	{
		vTaskDelay(2);
	}
	
	buzzer_off();
//	line_pid_param = origin_param;
}

/**
 * @brief: 长直立景点
 * @return {*}
 */
void view(void) // 打景点
{
	motor_all.Cspeed = Low_Speed;
	float origin_c = motor_all.Cincrement;
	float num;

	if(ScanMode == is_Front)
	{
		while (Infrared_ahead == 0) // 撞挡板
		{
			vTaskDelay(5);
		}
	}
	else if(ScanMode == is_Back)
	{
		while (Infrared_back == 0) // 撞挡板
		{
			vTaskDelay(5);
		}
	}

	motor_all.Cspeed = Rubbish_Speed;
	num = motor_all.Distance;
	while (fabsf(motor_all.Distance - num) < 12 * 60) // 前进一段距离 //7
	{
		vTaskDelay(5);
	}
	send_play_specified_command(29);
	CarBrake();
	vTaskDelay(500);
	motor_all.Cincrement = 0.05;
	Change_MODE();
	motor_all.Cspeed = nodesr.nowNode.speed;
	pid_mode_switch(is_Line);
	nodesr.nowNode.function = 0;
	motor_all.Cincrement = origin_c;
	nodesr.flag |= 0x04; // 到达路口
}

/**
 * @brief: 短直立景点
 * @return {*}
 */
void view1() // 打景点
{
	motor_all.Cspeed = Gyro_Speed;

	if(ScanMode == is_Front)
	{
		while (Infrared_ahead == 0) // 撞挡板
		{
			vTaskDelay(5);
		}
	}
	else if(ScanMode == is_Back)
	{
		while (Infrared_back == 0) // 撞挡板
		{
			vTaskDelay(5);
		}
	}

	float num = 0;
	num = motor_all.Distance;
	while (fabsf(motor_all.Distance - num) < 14 * 60) // 前进一段距离//420
	{
		vTaskDelay(5);
	}
	send_play_specified_command(1);
	CarBrake();
	
	nodesr.nowNode.function = 0;
	nodesr.flag |= 0x04; // 到达路口
}

/**
 * @brief:
 * @return {*}
 */
void back(void)
{
	char L = 0;
	char R = 0;

	pid_mode_switch(is_No);
	motor_all.Lspeed = motor_all.Rspeed = View_BACK_SPEED;
	infrare_open = 1;
	while ((L < 5) || (R < 5))
	{
		vTaskDelay(2);
		if (infrared.inside_outside_left)
			L++;
		if (infrared.inside_outside_right)
			R++;
	} // 检测到白线
	infrare_open = 0;
	buzzer_on();
	CarBrake();
	vTaskDelay(250);
	angle.AngleT = nodesr.nextNode.angle;
	FreeTurn(angle.AngleT, 3000, 3000);
	pid_mode_switch(is_Turn);
	while (fabs(need2turn(angle.AngleT, getAngleZ())) > 2)
	{ // 角度达到要求
		vTaskDelay(2);
		getline_error();
		if (Scaner.lineNum == 1 && ((Scaner.detail & 0x180) != 0) && (fabs(need2turn(angle.AngleT, getAngleZ())) < fabs(need2turn(angle.AngleT, nodesr.nowNode.angle)) * 0.15f))
		{
			break;
		}
	}
	buzzer_off();

	motor_all.Cspeed = 28;
	pid_mode_switch(is_Line);

	nodesr.nowNode.function = 0;
	nodesr.flag |= 0x04; // 到达路口
}
/*****************************************************************************
过波浪板
速度   ，  波浪板长度（cm）
长度主要是防止误判提前进入波浪板，然后提前放下造成严重失误。
先走这一段长度，保证能进入波浪板，且避免提前误判成已经走去波浪板。
*****************************************************************************/
void Barrier_WavedPlate(float lenght) // 波浪板长度
{
	lenght *= 63.3f;
	motor_all.Cspeed = Low_Speed;
	while(Scaner.ledNum <= 4 || Scaner.lineNum == 1)
	{
		vTaskDelay(2);
	}
	float num = 0;
	buzzer_on();
	struct PID_param origin_param = line_pid_param;
//	line_pid_param.kp = 23.5;
//	line_pid_param.ki = 0.004;
//	line_pid_param.kd = 0;
	motor_all.Cspeed = BL_Speed;
	pid_mode_switch(is_Line);
	num = motor_all.Distance;
	
	while (fabsf(motor_all.Distance - num) < lenght)
	{
		vTaskDelay(2);
		if((Scaner.detail & 0X07) != 0 && (Scaner.detail & 0XE000) == 0)
		{
			R_Fight = 1;
			L_Fight = 0;
			scaner_set.EdgeIgnore = 6;
		}
		else if((Scaner.detail & 0XE000) != 0 && (Scaner.detail & 0X07) == 0)
		{
			R_Fight = 0;
			L_Fight = 1;
			scaner_set.EdgeIgnore = 6;
		}
		else
		{
			R_Fight = 0;
			L_Fight = 0;
			scaner_set.EdgeIgnore = 0;
		}
	}
	line_pid_param = origin_param;
	nodesr.nowNode.function = 0;
	buzzer_off();
	R_Fight = 0;
	L_Fight = 0;
	nodesr.flag |= 0x04; // 到达路口
}

/**
 * @brief:
 * @param {float} length
 * @return {*}
 */
void South_Pole(float length)
{
	float num = 0;
	int UpSpeedTime = 0;
	float origin_turnM = motor_all.GyroT_speedMax;
	struct PID_param origin_param = line_pid_param;
	struct PID_param origin_param1 = gyroG_pid_param;
	motor_all.GyroT_speedMax = 25;

	gyroG_pid_param.kp = 0.5;
	gyroG_pid_param.ki = 0;
	gyroG_pid_param.kd = 0.5;
	
	encoder_clear();
	num = motor_all.Distance;
	while (fabsf(motor_all.Distance - num) < 60 * 40) // 前进一段距离//20
	{
		vTaskDelay(2);
		if (Scaner.ledNum >= 4 || Scaner.lineNum >= 2)
			break;
	}
	
	encoder_clear();
	angle.AngleG = nodesr.nowNode.angle;
	motor_all.Gspeed = Low_Speed;
	pid_mode_switch(is_Gyro);
	num = motor_all.Distance;
	while (fabsf(motor_all.Distance - num) < 2400)
	{
		if (imu.pitch > basic_p + 10)
		{
			break;
		} // 上坡
		vTaskDelay(2);
	}
	encoder_clear();
	motor_all.Cspeed = Mount_Speed;
	pid_mode_switch(is_Line);
	Rudder_control(170, 0); // 人站起来

	num = motor_all.Distance;
	buzzer_on();
	while (fabsf(motor_all.Distance - num) < 60*140) // 3400
	{
		vTaskDelay(2);
		if(Scaner.ledNum >= 10 && (fabsf(motor_all.Distance - num) > 60*50))
			break;
	}
	buzzer_off();

	angle.AngleG = getAngleZ();
	motor_all.Gspeed = IMPACT_SPEED;
	pid_mode_switch(is_Gyro);

	while ((ScanMode == is_Front && Infrared_ahead == 0) || (ScanMode == is_Back && Infrared_back == 0))
	{
		vTaskDelay(5);
	} // 撞挡板

	num = motor_all.Distance;
	while(fabsf(motor_all.Distance - num) < 23*60) // 前进一段距离
	{
		vTaskDelay(5);
	}

	CarBrake();
	vTaskDelay(250);
	
	mpuZreset(imu.yaw, nodesr.nowNode.angle); // 陀螺仪校正

	if(Clue_Stage[2] == 7 && nodesr.nowNode.nodenum == P8)
	{
		WaitForK210();
	}
	
	Arrived_Stage();

	num = motor_all.Distance;
	pid_mode_switch(is_No);
	motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;
	while (fabsf(motor_all.Distance - num) < 360)
	{
		vTaskDelay(2);
	}

	CarBrake();
	vTaskDelay(100);
	
	send_play_specified_command(34);

	Turn_Angle_Relative_Open(179);
	while (fabs(angle.AngleT - getAngleZ()) > 5)
	{
		vTaskDelay(5);
	}
	
	if((Clue_Stage[2] == 7 && nodesr.nowNode.nodenum == P8))
	{
		switch (Clue_Num[1])
		{
			case 0: send_play_specified_command(21); break;
			case 1: send_play_specified_command(19); break;
			case 2: send_play_specified_command(18); break;
			case 3: send_play_specified_command(15); break;
			case 4: send_play_specified_command(17); break;
			case 5: send_play_specified_command(12); break;
			case 6: send_play_specified_command(10); break;
			default: break;
		}
	}
	motor_pid_clear();
	
	if(map.routetime == 0)
		ConnectFirstBack();
	
	angle.AngleG = 0;
	motor_all.Gspeed = Rubbish_Speed;
	pid_mode_switch(is_Gyro);

//	motor_all.Cspeed = Rubbish_Speed;
//	pid_mode_switch(is_Line);
	
	Rudder_control(270, 0); // 人躺下
	while (imu.pitch > Down_pitch)
	{
		vTaskDelay(2);
		if(Scaner.ledNum >= 5)
			break;
	}

	line_pid_param.kp = 26;
	line_pid_param.ki = 0; 
	line_pid_param.kd = 1.5; 
	motor_all.Cspeed = Rubbish_Speed;
	pid_mode_switch(is_Line);
	
	buzzer_on();
	while(UpSpeedTime < 1000)
	{
		Cross_getline();
		if(Cross_Scaner.detail & 0X180)
			UpSpeedTime += 5;
		else
			UpSpeedTime +=1;
		vTaskDelay(2);
	}
	UpSpeedTime = 0;
	buzzer_off();
	
	motor_all.Cspeed = DownBHM_Speed;
	//pid_mode_switch(is_Line);
	
	while (imu.pitch < After_down)
	{
//		if((infrared.tail_left && infrared.tail_right)||(infrared.head_right && infrared.head_left))
//		{
//			break;
//		}
		vTaskDelay(2);
	}
	buzzer_off();
	motor_all.Cspeed = Low_Speed;
	pid_mode_switch(is_Line);
	line_pid_param = origin_param;
	gyroG_pid_param = origin_param1;
	motor_all.GyroT_speedMax = origin_turnM;
	nodesr.nowNode.function = 0;
	nodesr.flag |= 0x04; // 到达路口
}

/**
 * @brief:
 * @return {*}
 */
void QQB_1(void)
{
	int break_time = 0;
	float num;
	struct PID_param origin_param1 = gyroG_pid_param;

	motor_all.Cspeed = Low_Speed; // 给低速进入寻线//700//3000
	pid_mode_switch(is_Line);

	gyroG_pid_param.kp = 0.8; // 0.5 //0.01
	gyroG_pid_param.ki = 0;
	gyroG_pid_param.kd = 0; // 0.1

	if (nodesr.nowNode.nodenum == B9)
	{
		scaner_set.CatchsensorNum = line_weight[6]; // 给予左边权值
	}
	else if (nodesr.nowNode.nodenum == B8)
	{
		scaner_set.CatchsensorNum = line_weight[6]; // 给予左边权值
	}

	if (nodesr.nowNode.nodenum == B9)
	{
		num = motor_all.Distance;
		while (fabsf(motor_all.Distance - num) < 80 * 60)
		{
			vTaskDelay(2);
		}
		while ((imu.pitch < basic_p + 5) && (Scaner.ledNum <= 4 || Scaner.lineNum == 1))
		{
			vTaskDelay(2);
		}

		scaner_set.CatchsensorNum = 0;
		motor_all.Gspeed = 10;/*7*/
		angle.AngleG = /*-95*/getAngleZ()+25;
		pid_mode_switch(is_Gyro);
		while (fabs(need2turn(getAngleZ(), angle.AngleG)) > 5)
		{
			break_time++;
			if (imu.pitch >= 10 || break_time > 500)
				break;
			vTaskDelay(2);
		} // 转正位置

		gyroG_pid_param.kp = 8;
		gyroG_pid_param.ki = 0.004;
		gyroG_pid_param.kd = 0; // 0.1

		angle.AngleG = getAngleZ();
		motor_all.Gspeed = 9;
		encoder_clear();
		num = motor_all.Distance;
		while (fabsf(motor_all.Distance - num) < 30 * 60)
		{
			getline_error();
			buzzer_on();
			if(ScanMode == is_Front)
			{
				if ((infrared.head_left == 1 && infrared.head_right == 0) || (Scaner.detail & 0xF800))
				{
					angle.AngleG = getAngleZ() - 3;
				}
				else if ((infrared.head_left == 1 && infrared.head_right == 0) || (Scaner.detail & 0x3F))
				{
					angle.AngleG = getAngleZ() + 3;
				}
//				else
//				{
//					angle.AngleG = getAngleZ();
//				}
				vTaskDelay(2);
			}
			else if(ScanMode == is_Back)
			{
				if ((infrared.tail_left == 1 && infrared.tail_right == 0) || (Scaner.detail & 0xF800))
				{
					angle.AngleG = getAngleZ() - 3;
				}
				else if ((infrared.tail_left == 1 && infrared.tail_right == 0) || (Scaner.detail & 0x3F))
				{
					angle.AngleG = getAngleZ() + 3;
				}
//				else
//				{
//					angle.AngleG = getAngleZ();
//				}
				vTaskDelay(2);
			}
		}
		while (imu.pitch > basic_p - 5)
		{
			getline_error();
			buzzer_on();
			if(ScanMode == is_Front)
			{
				if ((infrared.head_left == 1 && infrared.head_right == 0) || (Scaner.detail & 0xF800))
				{
					angle.AngleG = getAngleZ() - 3;
				}
				else if ((infrared.head_left == 1 && infrared.head_right == 0) || (Scaner.detail & 0x3F))
				{
					angle.AngleG = getAngleZ() + 3;
				}
//				else
//				{
//					angle.AngleG = getAngleZ();
//				}
				vTaskDelay(2);
			}
			else if(ScanMode == is_Back)
			{
				if ((infrared.tail_left == 1 && infrared.tail_right == 0) || (Scaner.detail & 0xF800))
				{
					angle.AngleG = getAngleZ() - 3;
				}
				else if ((infrared.tail_left == 1 && infrared.tail_right == 0) || (Scaner.detail & 0x3F))
				{
					angle.AngleG = getAngleZ() + 3;
				}
//				else
//				{
//					angle.AngleG = getAngleZ();
//				}
				vTaskDelay(2);
			}
		}
		buzzer_off();
		infrare_open = 0;
		
		char angle1 = getAngleZ();
		num = motor_all.Distance;
		if(ScanMode == is_Front)
		{
			pid_mode_switch(is_Free);
			motor_set_pwm(1, -1000);
			motor_set_pwm(2, -1000);
			motor_set_pwm(3, 3000);
			motor_set_pwm(4, 3000);
		}
		else
		{
			pid_mode_switch(is_Free);
			motor_set_pwm(4, 1000);
			motor_set_pwm(3, 1000);
			motor_set_pwm(2, -3000);
			motor_set_pwm(1, -3000);
		}
		getline_error();
		while (fabsf(motor_all.Distance - num) < 15 * 100)
		{
			if (((fabsf(need2turn(getAngleZ(), angle1))) >= 50) || (Scaner.detail & 0X3C0))
			{
				break;
			}
			getline_error();
			vTaskDelay(2);
		}
		while ((fabsf(motor_all.Distance - num) < 10 * 100) || (Scaner.ledNum <= 2))
		{
			getline_error();
			vTaskDelay(3);
		}
	}
	if (nodesr.nowNode.nodenum == B8)
	{
		num = motor_all.Distance;
		while (fabsf(motor_all.Distance - num) < 80 * 60)
		{
			vTaskDelay(2);
		}
		while ((imu.pitch < basic_p + 5) && (Scaner.ledNum <= 4 || Scaner.lineNum == 1))
		{
			vTaskDelay(2);
		}
		scaner_set.CatchsensorNum = 0;
		motor_all.Gspeed = 10;
		angle.AngleG = getAngleZ()+25;
//		if(ScanMode == is_Back)
//			angle.AngleG = 90; // 78
//		else
//			angle.AngleG = 78;
		pid_mode_switch(is_Gyro);
		while (fabs(need2turn(getAngleZ(), angle.AngleG)) > 5)
		{
			break_time++;
			if (imu.pitch >= 10 || break_time > 500)
				break;
			vTaskDelay(2);
		} // 转正位置

		gyroG_pid_param.kp = 8;
		gyroG_pid_param.ki = 0.004;
		gyroG_pid_param.kd = 0;

		encoder_clear();
		motor_all.Gspeed = 9;
		angle.AngleG = getAngleZ();
		num = motor_all.Distance;
		infrare_open = 1;
		while (fabsf(motor_all.Distance - num) < 30 * 60)
		{
			vTaskDelay(2);
			getline_error();
			if(ScanMode == is_Front)
			{
				if ((infrared.head_left == 1 && infrared.head_right == 0) || (Scaner.detail & 0xF800))
				{
					angle.AngleG = getAngleZ() - 3;
				}
				else if ((infrared.head_left == 1 && infrared.head_right == 0) || (Scaner.detail & 0x3F))
				{
					angle.AngleG = getAngleZ() + 3;
				}
//				else
//				{
//					angle.AngleG = getAngleZ();
//				}
			}
			else if(ScanMode == is_Back)
			{
				if ((infrared.tail_left == 1 && infrared.tail_right == 0) || (Scaner.detail & 0xF800))
				{
					angle.AngleG = getAngleZ() - 3;
				}
				else if ((infrared.tail_left == 1 && infrared.tail_right == 0) || (Scaner.detail & 0x3F))
				{
					angle.AngleG = getAngleZ() + 3;
				}
//				else
//				{
//					angle.AngleG = getAngleZ();
//				}
			}
		}
		while (imu.pitch > basic_p - 5)//-5
		{
			vTaskDelay(2);
			getline_error();
			buzzer_on();
			if(ScanMode == is_Front)
			{
				if ((infrared.head_left == 1 && infrared.head_right == 0) || (Scaner.detail & 0xF800))
				{
					angle.AngleG = getAngleZ() - 3;
				}
				else if ((infrared.head_left == 1 && infrared.head_right == 0) || (Scaner.detail & 0x3F))
				{
					angle.AngleG = getAngleZ() + 3;
				}
//				else
//				{
//					angle.AngleG = getAngleZ();
//				}
			}
			else if(ScanMode == is_Back)
			{
				if ((infrared.tail_left == 1 && infrared.tail_right == 0) || (Scaner.detail & 0xF800))
				{
					angle.AngleG = getAngleZ() - 3;
				}
				else if ((infrared.tail_left == 1 && infrared.tail_right == 0) || (Scaner.detail & 0x3F))
				{
					angle.AngleG = getAngleZ() + 3;
				}
//				else
//				{
//					angle.AngleG = getAngleZ();
//				}
			}
		}
		infrare_open = 0;
		buzzer_off();

		CarBrake();
		vTaskDelay(500);
		float angle1 = getAngleZ();
		num = motor_all.Distance;
		if(ScanMode == is_Front)
		{
			pid_mode_switch(is_Free);
			motor_set_pwm(1, -2000); 
			motor_set_pwm(2, -2000);//37  //18
			motor_set_pwm(3, 3000);//40   //40
			motor_set_pwm(4, 3000);
		}
		else
		{	
			pid_mode_switch(is_Free);
			motor_set_pwm(4, 2000); 
			motor_set_pwm(3, 2000);
			motor_set_pwm(2, -3000);
			motor_set_pwm(1, -3000);
		}
		while (fabsf(motor_all.Distance - num) < 15 * 100)
		{
			if (((fabsf(need2turn(getAngleZ(), angle1))) >= 50) /*|| (Scaner.detail & 0x3c0)*/)
			{
				send_play_specified_command(34);
				break;
			}
			getline_error();
			vTaskDelay(2);
		}
	}
	if (nodesr.nowNode.nodenum == B8)
	{
		scaner_set.CatchsensorNum = line_weight[3];
		motor_all.Cspeed = QQB_Out_Speed;
	}
	else
	{
		scaner_set.CatchsensorNum = 0;
		motor_all.Cspeed = Low_Speed;
	}

	pid_mode_switch(is_Line);
	gyroG_pid_param = origin_param1;
	nodesr.nowNode.function = 0;
	nodesr.flag |= 0X04; // 到达路口
}

/**
 * @brief:
 * @return {*}
 */
void door(void)
{
	static uint8_t flag = 0;
	motor_all.Cspeed = nodesr.nowNode.speed;
	pid_mode_switch(is_Line);
	static int wait_cnt = 0;
	while (1)
	{
		if (Scaner.ledNum >= 8)
		{
			CarBrake();
			vTaskDelay(250); // 识别红绿
			color = 0;
			color_R = 0;
			if (ScanMode == is_Front) 
			{
				open_mv();
			}
			else
			{ 
				open_mvR();
			}

			while (color == 0 && color_R == 0)
			{
				vTaskDelay(2);
				wait_cnt++;
				if (wait_cnt == 500)
				{
					wait_cnt = 0;
					if (ScanMode == is_Front) 
					{
						open_mv();
					}
					else
					{ 
						open_mvR();
					}
				}
			}
			
			if (flag == 2) // 第三次不是绿灯    //N3-N8灯或者N5-N8灯判断
			{
				flag = 0;
				while (1)
				{
					color = 2;
					if (color == 2)
						send_play_specified_command(23);
					else if (color == 3)
						send_play_specified_command(33);
					else
						send_play_specified_command(31);
					
					if(nodesr.lastNode.nodenum == N3)
					{
						if(color == 3 || color == 2)
						{
							color_flag[2] = color;
							color_flag[3] = 1;
						}
						else
						{
							color_flag[2] = color;
							if(color_flag[0] == 3 && color_flag[1] == 3)
							{
								color_flag[3] = 2;
							}
						}
					}
					else
					{
						if(color == 3 || color == 2)
						{
							color_flag[1] = color;
							color_flag[0] = 1;
						}
						else
						{
							color_flag[1] = color;
							if(color_flag[2] == 3 && color_flag[3] == 3)
							{
								color_flag[0] = 2;
							}
						}
					}
					
					if(color == 3)
					{
						Change_MODE();
						
						if(Clue_Stage[1] == 5 && Clue_Stage[2] == 7)  //5 7
							route_reset(11);
						else if(Clue_Stage[1] == 5 && Clue_Stage[2] == 8)  //5 8
							route_reset(12);
						else if(Clue_Stage[1] == 6 && Clue_Stage[2] == 7)  //6 7
							route_reset(13);
						else if(Clue_Stage[1] == 6 && Clue_Stage[2] == 8)  //6 8
							route_reset(14);
						
						motor_all.Cspeed = nodesr.nowNode.speed;
						pid_mode_switch(is_Line);
					}
					else
					{
						if(Clue_Stage[1] == 5 && Clue_Stage[2] == 7)  //5 7
							route_reset(3);
						else if(Clue_Stage[1] == 5 && Clue_Stage[2] == 8)  //5 8
							route_reset(4);
						else if(Clue_Stage[1] == 6 && Clue_Stage[2] == 7)  //6 7
							route_reset(7);
						else if(Clue_Stage[1] == 6 && Clue_Stage[2] == 8)  //6 8
							route_reset(8);
					}
					
					close_mv();
					break;
				}
				motor_all.Cspeed = nodesr.nowNode.speed;
				pid_mode_switch(is_Line);
				nodesr.nowNode.function = 1;
				return;
			}
			if (flag == 1) // 第二次不是绿灯   //N5-N8灯或者N3-N8灯判断
			{
				while (1)
				{
					color = 3;
					if (color == 3 || color == 2)
					{
						if(nodesr.lastNode.nodenum == N5)
							color_flag[1] = color;
						else
							color_flag[2] = color;
						
						if (color == 3)
							send_play_specified_command(33);
						else
							send_play_specified_command(23);

						Change_MODE();
						
						if(Clue_Stage[1] == 5)
							route_reset(9);
						else
							route_reset(10);
						
						motor_all.Cspeed = nodesr.nowNode.speed;
						pid_mode_switch(is_Line);
						flag = 2;
						
						close_mv();
						return;
					}
					else if (color == 1)
					{
						if(nodesr.lastNode.nodenum == N5)
							color_flag[1] = color;
						else
							color_flag[2] = color;
						
						send_play_specified_command(31);
						motor_all.Cspeed = nodesr.nowNode.speed;
						pid_mode_switch(is_Line);
						
						if(Clue_Stage[1] == 5 && Clue_Stage[2] == 7)  //5 7
							route_reset(3);
						else if(Clue_Stage[1] == 5 && Clue_Stage[2] == 8)  //5 8
							route_reset(4);
						else if(Clue_Stage[1] == 6 && Clue_Stage[2] == 7)  //6 7
							route_reset(7);
						else if(Clue_Stage[1] == 6 && Clue_Stage[2] == 8)  //6 8
							route_reset(8);
						
						flag = 0;
						close_mv();
						return;
					}
				}
			}
			if (flag == 0) // 第一次  //N5-N12灯或者N3-N10灯判断
			{
				while (1)
				{
					color = 3;
					if (color == 3 || color == 2)
					{
						if(nodesr.nowNode.nodenum == N12)
							color_flag[0] = color;
						else
							color_flag[3] = color;
						
						if (color == 3)
							send_play_specified_command(33);
						else
							send_play_specified_command(23);
						
						map.point -= 2;
						route[map.point] = N8;

						Change_MODE();
						
						if(Clue_Stage[1] == 5)
							nodesr.nowNode = Node[getNextConnectNode(N12, N5)]; // 重新设置nowNode
						else
							nodesr.nowNode = Node[getNextConnectNode(N10, N3)]; // 重新设置nowNode
						
						nodesr.nowNode.step = 90 * 60;						// 100
						nodesr.nowNode.flag |= DRIGHT | DLEFT;
						nodesr.nowNode.speed = 60;
						pid_mode_switch(is_Line);
						nodesr.flag |= 0x20;
						flag = 1;
						close_mv();
						return;
					}
					else if (color == 1) // 绿
					{
						if(nodesr.nowNode.nodenum == N12)
							color_flag[0] = color;
						else
							color_flag[3] = color;
						
						send_play_specified_command(31);
						
						if(Clue_Stage[1] == 5 && Clue_Stage[2] == 7)  //5 7
							route_reset(1);
						else if(Clue_Stage[1] == 5 && Clue_Stage[2] == 8)  //5 8
							route_reset(2);
						else if(Clue_Stage[1] == 6 && Clue_Stage[2] == 7)  //6 7
							route_reset(5);
						else if(Clue_Stage[1] == 6 && Clue_Stage[2] == 8)  //6 8
							route_reset(6);
						
						
						flag = 0;
						close_mv();
						return;
					}
				}
			}
		}
	}
}

/**
 * @brief:
 * @param {u8} flag
 * @return {*}
 */
void route_reset(u8 flag) // 平台三下来
{
	static u8 temp = 0, i = 0;
	temp = map.point;
	i = 0;
	if (flag == 1) 
	{
		while (1)
		{
			route[temp++] = door1route[i++]; // 路线连接
			if (door1route[i] == 255)
			{
				route[temp] = door1route[i];
				nodesr.nowNode = Node[getNextConnectNode(N5, N12)]; // 重新设置nowNode
				nodesr.nowNode.flag = DLEFT | LEFT_LINE;
				nodesr.nowNode.step = 80 * 60;
				nodesr.nowNode.function = 1;
				nodesr.flag |= 0x80;
				break;
			}
		}
	}
	else if (flag == 2) 
	{
		while (1)
		{
			route[temp++] = door2route[i++]; // 路线连接
			if (door2route[i] == 255)
			{
				route[temp] = door2route[i];
				nodesr.nowNode = Node[getNextConnectNode(N5, N12)]; // 重新设置nowNode
				nodesr.nowNode.flag = DLEFT | LEFT_LINE;
				nodesr.nowNode.step = 80 * 60;
				nodesr.nowNode.function = 1;
				nodesr.flag |= 0x80;
				break;
			}
		}
	}
	else if (flag == 3)
	{
		while (1)
		{
			route[temp++] = door3route[i++]; // 路线连接
			if (door3route[i] == 255)
			{
				route[temp] = door3route[i];
				if(nodesr.lastNode.nodenum == N5)
				{
					nodesr.nowNode = Node[getNextConnectNode(N5, N8)]; // 重新设置nowNode
					nodesr.nowNode.flag = DLEFT | DRIGHT;
					nodesr.nowNode.step = 20 * 60; // 70
					nodesr.nowNode.function = 1;
					nodesr.flag |= 0x80;
				}
				else
				{
					nodesr.nowNode = Node[getNextConnectNode(N3, N8)]; // 重新设置nowNode
					nodesr.nowNode.flag = DLEFT | DRIGHT;
					nodesr.nowNode.step = 20 * 60; // 70
					nodesr.nowNode.function = 1;
					nodesr.flag |= 0x80;
				}
				break;
			}
		}
	}
	else if (flag == 4)
	{
		while (1)
		{
			route[temp++] = door4route[i++]; // 路线连接
			if (door4route[i] == 255)
			{
				route[temp] = door4route[i];
				if(nodesr.lastNode.nodenum == N5)
				{
					nodesr.nowNode = Node[getNextConnectNode(N5, N8)]; // 重新设置nowNode
					nodesr.nowNode.flag = DLEFT | DRIGHT;
					nodesr.nowNode.step = 20 * 60; // 70
					nodesr.nowNode.function = 1;
					nodesr.flag |= 0x80;
				}
				else
				{
					nodesr.nowNode = Node[getNextConnectNode(N3, N8)]; // 重新设置nowNode
					nodesr.nowNode.flag = DLEFT | DRIGHT;
					nodesr.nowNode.step = 20 * 60; // 70
					nodesr.nowNode.function = 1;
					nodesr.flag |= 0x80;
				}
				break;
			}
		}
	}
	else if (flag == 5)
	{
		while (1)
		{
			route[temp++] = door5route[i++]; // 路线连接
			if (door5route[i] == 255)
			{
				route[temp] = door5route[i];
				nodesr.nowNode = Node[getNextConnectNode(N3, N10)]; // 重新设置nowNode
				nodesr.nowNode.flag = DRIGHT | RIGHT_LINE;
				nodesr.nowNode.step = 80 * 60;
				nodesr.nowNode.function = 1;
				nodesr.flag |= 0x80;
				break;
			}
		}
	}
	else if (flag == 6)
	{

		while (1)
		{
			route[temp++] = door6route[i++]; // 路线连接
			if (door6route[i] == 255)
			{
				route[temp] = door6route[i];
				nodesr.nowNode = Node[getNextConnectNode(N3, N10)]; // 重新设置nowNode
				nodesr.nowNode.flag = DRIGHT | RIGHT_LINE;
				nodesr.nowNode.step = 80 * 60;
				nodesr.nowNode.function = 1;
				nodesr.flag |= 0x80;
				break;
			}
		}
	}
	else if (flag == 7)
	{
		while (1)
		{
			route[temp++] = door7route[i++]; // 路线连接
			if (door7route[i] == 255)
			{
				route[temp] = door7route[i];
				if(nodesr.lastNode.nodenum == N5)
				{
					nodesr.nowNode = Node[getNextConnectNode(N5, N8)]; // 重新设置nowNode
					nodesr.nowNode.flag = DLEFT | DRIGHT;
					nodesr.nowNode.step = 20 * 60; // 70
					nodesr.nowNode.function = 1;
					nodesr.flag |= 0x80;
				}
				else
				{
					nodesr.nowNode = Node[getNextConnectNode(N3, N8)]; // 重新设置nowNode
					nodesr.nowNode.flag = DLEFT | DRIGHT;
					nodesr.nowNode.step = 20 * 60; // 70
					nodesr.nowNode.function = 1;
					nodesr.flag |= 0x80;
				}
				break;
			}
		}
	}
	else if (flag == 8)
	{
		while (1)
		{
			route[temp++] = door8route[i++]; // 路线连接
			if (door8route[i] == 255)
			{
				route[temp] = door8route[i];
				if(nodesr.lastNode.nodenum == N5)
				{
					nodesr.nowNode = Node[getNextConnectNode(N5, N8)]; // 重新设置nowNode
					nodesr.nowNode.flag = DLEFT | DRIGHT;
					nodesr.nowNode.step = 20 * 60; // 70
					nodesr.nowNode.function = 1;
					nodesr.flag |= 0x80;
				}
				else
				{
					nodesr.nowNode = Node[getNextConnectNode(N3, N8)]; // 重新设置nowNode
					nodesr.nowNode.flag = DLEFT | DRIGHT;
					nodesr.nowNode.step = 20 * 60; // 70
					nodesr.nowNode.function = 1;
					nodesr.flag |= 0x80;
				}
				break;
			}
		}
	}
	else if (flag == 9)
	{
		while (1)
		{
			route[temp++] = door9route[i++]; // 路线连接
			if (door9route[i] == 255)
			{
				route[temp] = door9route[i];
				nodesr.nowNode = Node[getNextConnectNode(N8, N5)]; // 重新设置nowNode
				nodesr.nowNode.function = 1;
				nodesr.nowNode.step = 50 * 60;
				nodesr.flag |= 0x80;
				break;
			}
		}
	}
	else if (flag == 10)
	{
		while (1)
		{
			route[temp++] = door10route[i++]; // 路线连接
			if (door10route[i] == 255)
			{
				route[temp] = door10route[i];
				nodesr.nowNode = Node[getNextConnectNode(N8, N3)]; // 重新设置nowNode
				nodesr.nowNode.function = 1;
				nodesr.nowNode.step = 50 * 60;
				nodesr.flag |= 0x80;
				break;
			}
		}
	}
	else if (flag == 11)
	{
		while (1)
		{
			route[temp++] = door11route[i++]; // 路线连接
			if (door11route[i] == 255)
			{
				route[temp] = door11route[i];
				nodesr.nowNode = Node[getNextConnectNode(N8, N3)]; // 重新设置nowNode
				nodesr.nowNode.function = 1;
				nodesr.nowNode.step = 50 * 60;
				nodesr.flag |= 0x80;
				Node[getNextConnectNode(N3, N10)].function = 1;
				Node[getNextConnectNode(N3, N10)].step = 230 * 60;
				Node[getNextConnectNode(N3, N10)].speed = 70;
				Node[getNextConnectNode(N3, N10)].flag |= SLOWDOWN;
				break;
			}
		}
	}
	else if (flag == 12)
	{
		while (1)
		{
			route[temp++] = door12route[i++]; // 路线连接
			if (door12route[i] == 255)
			{
				route[temp] = door12route[i];
				nodesr.nowNode = Node[getNextConnectNode(N8, N3)]; // 重新设置nowNode
				nodesr.nowNode.function = 1;
				nodesr.nowNode.step = 50 * 60;
				nodesr.flag |= 0x80;
				Node[getNextConnectNode(N3, N10)].function = 1;
				Node[getNextConnectNode(N3, N10)].step = 230 * 60;
				Node[getNextConnectNode(N3, N10)].speed = 70;
				Node[getNextConnectNode(N3, N10)].flag |= SLOWDOWN;
				break;
			}
		}
	}
	else if (flag == 13)
	{
		while (1)
		{
			route[temp++] = door13route[i++]; // 路线连接
			if (door13route[i] == 255)
			{
				route[temp] = door13route[i];
				nodesr.nowNode = Node[getNextConnectNode(N8, N5)]; // 重新设置nowNode
				nodesr.nowNode.function = 1;
				nodesr.nowNode.step = 50 * 60;
				nodesr.flag |= 0x80;
				Node[getNextConnectNode(N5, N12)].function = 1;
				Node[getNextConnectNode(N5, N12)].step = 230 * 60;
				Node[getNextConnectNode(N5, N12)].speed = 70;
				Node[getNextConnectNode(N5, N12)].flag |= SLOWDOWN;
				break;
			}
		}
	}
	else if (flag == 14)
	{
		while (1)
		{
			route[temp++] = door14route[i++]; // 路线连接
			if (door14route[i] == 255)
			{
				route[temp] = door14route[i];
				nodesr.nowNode = Node[getNextConnectNode(N8, N5)]; // 重新设置nowNode
				nodesr.nowNode.function = 1;
				nodesr.nowNode.step = 50 * 60;
				nodesr.flag |= 0x80;
				Node[getNextConnectNode(N5, N12)].function = 1;
				Node[getNextConnectNode(N5, N12)].step = 230 * 60;
				Node[getNextConnectNode(N5, N12)].speed = 70;
				Node[getNextConnectNode(N5, N12)].flag |= SLOWDOWN;
				break;
			}
		}
	}
}

/**
 * @brief:
 * @return {*}
 */
void undermou(void)
{
	float num = 0;
	while (imu.pitch > -3)
		;
	motor_all.Cspeed = 60;
	num = motor_all.Distance;
	while (fabsf(motor_all.Distance - num) < 50)
		;
	buzzer_on();
	if (nodesr.nowNode.nodenum == N14)
	{
		motor_all.Cspeed = 110;
		num = motor_all.Distance;
		while (fabsf(motor_all.Distance - num) < 100)
			;
		motor_all.Cspeed = 60;
	}
	while (!deal_arrive())
		;
	nodesr.nowNode.function = 0;
	nodesr.flag |= 0x04; // 到达路口
}

/**
 * @brief:
 * @return {*}
 */
void S_curve(void)
{
	float num;
	struct PID_param origin_param = line_pid_param;

	num = motor_all.Distance;
	motor_all.Cspeed = nodesr.nowNode.speed;

	if (nodesr.nextNode.nodenum == N13)
	{
		motor_all.Cspeed = 110;
		while (fabsf(motor_all.Distance - num) < 110)
			;
		motor_all.Cspeed = 60;
		scaner_set.CatchsensorNum = line_weight[11]; // C1->C2
		line_pid_param.kp = 40;
		while (fabsf(getAngleZ() - nodesr.nextNode.angle) > 10)
			;
	}
	else if (nodesr.nextNode.nodenum == C1)
	{
		motor_all.Cspeed = 110;
		while (fabsf(motor_all.Distance - num) < 60)
			;
		scaner_set.CatchsensorNum = line_weight[5];
		line_pid_param.kp = 70;
		while (fabsf(getAngleZ() - nodesr.nextNode.angle) > 10)
			;
	}

	line_pid_param = origin_param;
	scaner_set.CatchsensorNum = 0;

	nodesr.nowNode.function = 0;
	nodesr.flag |= 0x04; // 到达路口
}

/**
 * @brief:
 * @return {*}
 */
void ignore_node(void)
{
	nodesr.nowNode.function = 0;
	nodesr.flag |= 0x04; // 到达路口
}

/**
 * @brief:
 * @return {*}
 */
//void get_newroute(void)
//{
//	mapInit1();
//	map.point = 0;
//	for (int i = 0; i < 126; i++)
//	{
//		if (Node[i].function == DOOR)
//		{
//			Node[i].step *= 2;
//			Node[i].function = 1;
//			Node[i].speed = 40; // 56
//			Node[i].flag |= SLOWDOWN;
//		}
//	}
//	if (color_flag[0] == 1) // 第一个门开
//	{
//		u8 temp[100] = {B1, N1, P1, N1, B2, N4, N5, N6, S2, N6, P4, N6, N5, N12, P6, N13, N12, N16, S5, N16, N18, B5, N19, C6, B7, N22, C9, G1, P8, G1, C9, N22, B6, N20, P7, N20, C4, C8, C7, N14, C3, N9, B9, N7, P5, N7, B8, N9, N10, N15, S4, N15, N10, N11, N12, N5, N4, P3, N3, S1, N3, N4, B3, N2, P2, 0XFF};
//		for (int i = 0; i < 100; i++)
//		{
//			route[i] = temp[i];
//			if (temp[i] == 0xff)
//				break;
//		}
//	}
//	if (color_flag[0] == 2 & color_flag[3] == 1)
//	{
//		u8 temp[100] = {B1, N1, P1, N1, B2, N4, N5, N6, S2, N6, P4, N6, N5, N12, P6, N13, N12, N16, S5, N16, N18, B5, N19, C6, B7, N22, C9, G1, P8, G1, C9, N22, B6, N20, P7, N20, C4, C8, C7, N14, C3, N9, B9, N7, P5, N7, B8, N9, N10, N15, S4, N15, N10, N3, S1, N3, P3, N3, N4, B3, N2, P2, 0XFF};
//		for (int i = 0; i < 100; i++)
//		{
//			route[i] = temp[i];
//			if (temp[i] == 0xff)
//				break;
//		}
//	}
//	if (color_flag[0] == 2 & color_flag[3] == 3 & color_flag[2] == 1)
//	{
//		u8 temp[100] = {B1, N1, P1, N1, B2, N4, N5, N6, S2, N6, P4, N6, N5, N12, P6, N13, N12, N16, S5, N16, N18, B5, N19, C6, B7, N22, C9, G1, P8, G1, C9, N22, B6, N20, P7, N20, C4, C8, C7, N14, C3, N9, B9, N7, P5, N7, B8, N9, N10, N15, S4, N15, N10, N8, N3, S1, N3, P3, N3, N4, B3, N2, P2, 0XFF};
//		for (int i = 0; i < 100; i++)
//		{
//			route[i] = temp[i];
//			if (temp[i] == 0xff)
//				break;
//		}
//	}
//	if (color_flag[0] == 2 & color_flag[3] == 3 & color_flag[2] == 3)
//	{
//		u8 temp[100] = {B1, N1, P1, N1, B2, N4, N5, N6, S2, N6, P4, N6, N5, N12, P6, N13, N12, N16, S5, N16, N18, B5, N19, C6, B7, N22, C9, G1, P8, G1, C9, N22, B6, N20, P7, N20, C4, C8, C7, N14, C3, N9, B9, N7, P5, N7, B8, N9, N10, N15, S4, N15, N10, N8, N5, N4, P3, N3, S1, N3, N4, B3, N2, P2, 0XFF};
//		for (int i = 0; i < 100; i++)
//		{
//			route[i] = temp[i];
//			if (temp[i] == 0xff)
//				break;
//		}
//	}
//	if (color_flag[0] == 3 & color_flag[1] == 1)
//	{
//		u8 temp[100] = {B1, N1, P1, N1, B2, N4, N5, N6, S2, N6, P4, N6, N5, N8, N12, P6, N13, N12, N16, S5, N16, N18, B5, N19, C6, B7, N22, C9, G1, P8, G1, C9, N22, B6, N20, P7, N20, C4, C8, C7, N14, C3, N9, B9, N7, P5, N7, B8, N9, N10, N15, S4, N15, N10, N8, N5, N4, P3, N3, S1, N3, N4, B3, N2, P2, 0XFF};
//		for (int i = 0; i < 100; i++)
//		{
//			route[i] = temp[i];
//			if (temp[i] == 0xff)
//				break;
//		}
//	}
//	if (color_flag[0] == 3 & color_flag[1] == 2 & color_flag[3] == 1)
//	{
//		u8 temp[100] = {B1, N1, P1, N1, B2, N4, N5, N6, S2, N6, P4, N6, N5, N8, N12, P6, N13, N12, N16, S5, N16, N18, B5, N19, C6, B7, N22, C9, G1, P8, G1, C9, N22, B6, N20, P7, N20, C4, C8, C7, N14, C3, N9, B9, N7, P5, N7, B8, N9, N10, N15, S4, N15, N10, N3, S1, N3, P3, N3, N4, B3, N2, P2, 0XFF};
//		for (int i = 0; i < 100; i++)
//		{
//			route[i] = temp[i];
//			if (temp[i] == 0xff)
//				break;
//		}
//	}
//	if (color_flag[0] == 3 & color_flag[1] == 2 & color_flag[3] == 3)
//	{
//		u8 temp[100] = {B1, N1, P1, N1, B2, N4, N5, N6, S2, N6, P4, N6, N5, N8, N12, P6, N13, N12, N16, S5, N16, N18, B5, N19, C6, B7, N22, C9, G1, P8, G1, C9, N22, B6, N20, P7, N20, C4, C8, C7, N14, C3, N9, B9, N7, P5, N7, B8, N9, N10, N15, S4, N15, N10, N8, N3, S1, N3, P3, N3, N4, B3, N2, P2, 0XFF};
//		for (int i = 0; i < 100; i++)
//		{
//			route[i] = temp[i];
//			if (temp[i] == 0xff)
//				break;
//		}
//	}
//	if (color_flag[0] == 3 & color_flag[1] == 3 & color_flag[2] == 1) // 从最外面出去吧
//	{
//		u8 temp[100] = {B1, N1, P1, N1, B2, N4, N5, N6, S2, N6, P4, N6, N5, N4, P3, N3, S1, N3, N8, N12, P6, N13, N12, N16, S5, N16, N18, B5, N19, C6, B7, N22, C9, G1, P8, G1, C9, N22, B6, N20, P7, N20, C4, C8, C7, N14, C3, N9, B9, N7, P5, N7, B8, N9, N10, N15, S4, N15, N10, N8, N3, N4, B3, N2, P2, 0XFF};
//		//	u8 temp[100]={B1,P1,N1,B2,N4,N5,N6,P4,N6,S2,N6,C1,C2,N13,P6,N13,N12,N16,S5,N16,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N15,S4,N15,N10,N8,N3,N4,B3,N2,P2,0XFF};

//		for (int i = 0; i < 100; i++)
//		{
//			route[i] = temp[i];
//			if (temp[i] == 0xff)
//				break;
//		}
//	}
//	if (color_flag[0] == 3 & color_flag[1] == 3 & color_flag[2] == 2) // 从最外面出去吧
//	{
//		u8 temp[100] = {B1, N1, P1, N1, B2, N4, N5, N6, S2, N6, P4, N6, N5, N4, P3, N3, S1, N3, N8, N12, P6, N13, N12, N16, S5, N16, N18, B5, N19, C6, B7, N22, C9, G1, P8, G1, C9, N22, B6, N20, P7, N20, C4, C8, C7, N14, C3, N9, B9, N7, P5, N7, B8, N9, N10, N15, S4, N15, N10, N3, N4, B3, N2, P2, 0XFF};
//		//	u8 temp[100]={B1,P1,N1,B2,N4,N5,N6,P4,N6,S2,N6,C1,C2,N13,P6,N13,N12,N16,S5,N16,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N15,S4,N15,N10,N3,P3,N3,S1,N3,N4,B3,N2,P2,0XFF};
//		for (int i = 0; i < 100; i++)
//		{
//			route[i] = temp[i];
//			if (temp[i] == 0xff)
//				break;
//		}
//	}
//}


void get_newroute(void)
{
	mapInit1();
	map.point = 0;
	for (int i = 0; i < 126; i++)
	{
		if (Node[i].function == DOOR)
		{
			Node[i].step *= 2;
			Node[i].function = 1;
			Node[i].speed = 60;
			Node[i].flag |= SLOWDOWN;
		}
	}
	if (color_flag[0] == 1)//直接走4
	{
		u8 temp[100] = {B1, N1, P1, N1, B2, N4, N5, N6, P4, N6, S2, N6, N5,N12,P6,N13,N12,N16,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,S3,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N15,S4,N15,C5,N18,N16,S5,N16,N12,N5,N4,N3,P3,N3,S1,N3,N4,B3,N2,P2, 0XFF};
		for (int i = 0; i < 100; i++)
		{
			route[i] = temp[i];
			if (temp[i] == 0xff)
				break;
		}
	}
	else if (color_flag[1] == 1)//3
	{
		if (color_flag[3] == 2)//1是黄，走1去3回
		{
			u8 temp[100] = {B1, N1, P1, N1, B2, N4, N3,P3,N3,S1,N3,N10,N15,S4,N15,N10,N9,B9,N7,P5,N7,B8,N9,C3,N14,S3,N14,C7,C8,C4,N20,P7,N20,B6,N22,C9,G1,P8,G1,C9,N22,B7,C6,N19,B5,N18,N16,S5,N16,N12,P6,N13,N12,N8,N5,N6,P4,N6,S2,N6,N5,N4,B3,N2,P2,0XFF};
			for (int i = 0; i < 100; i++)
			{
				route[i] = temp[i];
				if (temp[i] == 0xff)
					break;
			}
		}
		else if (color_flag[2] == 2)//2是黄，走2去3回
		{
			u8 temp[100] = {B1, N1, P1, N1, B2, N4, N3,P3, N3, S1, N3, N8, N12, P6, N13, N12, N16, S5, N16, N18, B5, N19, C6, B7, N22, C9, G1, P8, G1, C9, N22, B6, N20, P7, N20, C4, C8, C7, N14, S3, N14,C3, N9, B9, N7, P5, N7, B8, N9, N10, N15, S4, N15, N10,N8, N5, N6, P4, N6, S2,N6,N5, N4, B3, N2, P2, 0xFF};
			for (int i = 0; i < 100; i++)
			{
				route[i] = temp[i];
				if (temp[i] == 0xff)
					break;
			}
		} 	 	
		else if (color_flag[0] == 2)//4是黄，4去3回，记得写12红3绿 4一定可以过的情况(赋值为黄）
		{ 		
			u8 temp[100] = {B1, N1, P1, N1, B2, N4, N5, N6, P4, N6, S2, N6, N5,N12,P6,N13,N12,N16,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,S3,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N15,S4,N15,C5,N18,N16,S5,N16,N12,N8,N5,N4,N3,P3,N3,S1,N3,N4,B3,N2,P2,0XFF};
			for (int i = 0; i < 100; i++)
			{
				route[i] = temp[i];
				if (temp[i] == 0xff)
					break;
			}
		}
		else
		{
			u8 temp[100] = {B1, N1, P1, N1, B2, N4, N5, N6, P4, N6, S2, N6, N5,N8,N10,N15,S4,N15,N10,N9,B9,N7,P5,N7,B8,N9,C3,N14,S3,N14,C7,C8,C4,N20,P7,N20,B6,N22,C9,G1,P8,G1,C9,N22,B7,C6,N19,B5,N18,N16,S5,N16,N12,P6,N13,N12,N8,N5,N4,N3,P3,N3,S1,N3,N4,B3,N2,P2, 0XFF};
			//temp[100] = {B1, N1, P1, N1, B2, N4, P3,N3,S1,N3,N8,N12,P6,N13,N12,N16,S5,N16,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,S3,C3,N9,B9,N7,P5,N7,B8,N9,N10,N15,S4,N15,C5,N18,N16,S5,N16,N12,N8,N3,N4,N5,N6,P4,N6,N5,N4,B3,N2,P2,0XFF};
			for (int i = 0; i < 100; i++)
			{
				route[i] = temp[i];
				if (temp[i] == 0xff)
					break;
			}
		}
	}
	else if (color_flag[2] == 1)//2
	{
		if (color_flag[3] == 2)//1是黄，1去2回，记得写34红赋值1黄
		{
			u8 temp[100] = {B1, N1, P1, N1, B2, N4, N3,P3,N3,N10,N9,B9,N7,P5,N7,B8,N9,C3,N14,S3,N14,C7,C8,C4,N20,P7,N20,B6,N22,C9,G1,P8,G1,C9,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N16,S5,N16,N18,C5,N15,S4,N15,N10,N8,N3,S1,N3,N4,N5,N6,P4,N6,S2,N6,N5,N4,B3,N2,P2,0XFF};
			for (int i = 0; i < 100; i++)
			{
				route[i] = temp[i];
				if (temp[i] == 0xff)
					break;
			}
		}
		else if (color_flag[1] == 2)//3是黄，证明宝藏在P6/P4
		{
			u8 temp[100] = {B1, N1, P1, N1, B2, N4, N5, N6, P4, N6, S2, N6, N5,N8,N10,N15,S4,N15,N10,N9,B9,N7,P5,N7,B8,N9,C3,N14,S3,N14,C7,C8,C4,N20,P7,N20,B6,N22,C9,G1,P8,G1,C9,N22,B7,C6,N19,B5,N18,N16,S5,N16,N12,P6,N13,N12,N8,N3,S1,N3,P3,N3,N4,B3,N2,P2,0XFF};
			for (int i = 0; i < 100; i++)
			{
				route[i] = temp[i];
				if (temp[i] == 0xff)
					break;
			}
		}
		else if (color_flag[0] == 2)//4是黄，证明宝藏在P6/P4
		{
			u8 temp[100] = {B1, N1, P1, N1, B2, N4, N5, N6, P4, N6, S2, N6, N5,N12,P6,N13,N12,N16,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,S3,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N15,S4,N15,C5,N18,N16,S5,N16,N12,N8,N3,S1,N3,P3,N3,N4,B3,N2,P2,0XFF};
			for (int i = 0; i < 100; i++)
			{
				route[i] = temp[i];
				if (temp[i] == 0xff)
					break;
			}
		}
		else
		{
			u8 temp[100] = {B1, N1, P1, N1, B2, N4, N3,P3, N3, S1, N3, N8, N12, P6, N13, N12, N16,S5,N16, N18, B5, N19, C6, B7, N22, C9, G1, P8, G1, C9, N22, B6, N20, P7, N20, C4, C8, C7, N14, S3,N14, C3, N9, B9, N7, P5, N7, B8, N9, N10, N15, S4, N15, N10,N8,N3, N4, N5, N6, P4, N6,S2,N6,N5, N4, B3, N2, P2, 0xFF};
			for (int i = 0; i < 100; i++)
			{
				route[i] = temp[i];
				if (temp[i] == 0xff)
					break;
			}
		}
	}
	else if (color_flag[3] == 1)//直接走1
	{
		u8 temp[100] = {B1, N1, P1, N1, B2, N4, N3,P3,N3,S1,N3,N10,N9,B9,N7,P5,N7,B8,N9,C3,N14,S3,N14,C7,C8,C4,N20,P7,N20,B6,N22,C9,G1,P8,G1,C9,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N16,S5,N16,N18,C5,N15,S4,N15,N10,N3,N4,N5,N6,P4,N6,S2,N6,N5,N4,B3,N2,P2,0XFF};
		for (int i = 0; i < 100; i++)
		{
			route[i] = temp[i];
			if (temp[i] == 0xff)
				break;
		}
	}
}

void Connect_Route(void)
{
	static u8 temp = 0, i = 0;
	temp = map.point+2;
	i = 0;
	if(Clue_Stage[1] == 5)
	{
		while (1)
		{
			route[temp++] = Connect5Route[i++]; // 路线连接
			if (Connect5Route[i] == 255)
			{
				route[temp] = Connect5Route[i];
				break;
			}
		}
	}
	else if(Clue_Stage[1] == 6)
	{
		while (1)
		{
			route[temp++] = Connect6Route[i++]; // 路线连接
			if (Connect6Route[i] == 255)
			{
				route[temp] = Connect6Route[i];
				break;
			}
		}
	}
}

void WaitForK210(void)
{
	Clue_Num[0] = 3;
	Clue_Num[1] = 3;
	int break_times = 0;
	float num = 0;
	CarBrake();
	vTaskDelay(1000);
	vTaskDelay(1000);
	vTaskDelay(1000);
	vTaskDelay(1000);
	vTaskDelay(500);
	if((Clue_Stage[1] == 5 && nodesr.nowNode.nodenum == P6)
	|| (Clue_Stage[1] == 6 && nodesr.nowNode.nodenum == P5)
	|| (Clue_Stage[2] == 7 && nodesr.nowNode.nodenum == P8)
	|| (Clue_Stage[2] == 8 && nodesr.nowNode.nodenum == P7))
	{
//		open_K210();
//		vTaskDelay(500);
//		while(K210_Rece == 0)
//		{
//			vTaskDelay(2);
//			break_times++;
//			if (break_times >= 500)
//			{
//				num = motor_all.Distance;
//				motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;
//				while (fabsf(motor_all.Distance - num) < 60) // 360//60
//				{
//					vTaskDelay(2);
//				}
//				CarBrake();
//				encoder_clear();
//				break_times = 0;
//			}
//			if(Clue_Num[0] == 6)
//				break;
//		}
		if(Clue_Num[0] != 0)
		{
			if(Clue_Num[0] == 6)
			{
				Clue_Num[1] = 0;
			}
			if(Clue_Num[1] == 6)
			{
				Clue_Num[1] = 0;
			}
		}
		if(nodesr.nowNode.nodenum == P8 || nodesr.nowNode.nodenum == P7)
		{
			treasure = Clue_Num[0] + Clue_Num[1];
			switch (treasure)
			{
				case 2: treasure = P1; break;
				case 3: treasure = P3; break;
				case 4: treasure = P4; break;
				case 5: treasure = P6; break;
				case 6: treasure = P5; break;
				default: break;
			}
		}
		close_K210();
	}
}

/**
 * @brief:
 * @return {*}
 */
void zhunbei(void)
{	 
//	send_play_specified_command(29);
//	while(1)
//	{
//	Turn_Angle_Relative_Open(179);

//	while (fabsf(need2turn(angle.AngleT, getAngleZ())) > 4) // 7
//	{
//		vTaskDelay(2);
//	}
//	CarBrake();
//	vTaskDelay(1000);
//}
//	
//  motor_all.Cspeed=60;
//	scaner_set.CatchsensorNum = 0;
//	pid_mode_switch(is_Line);
//	vTaskDelay(1000);
//	vTaskDelay(1000);
//	vTaskDelay(1000);
//	motor_all.Cspeed=0;
//	while(1)
//		vTaskDelay(2);
//	Change_MODE();
	
//	CarBrake();
//	vTaskDelay(500);
//		pid_mode_switch(is_Line);
//	motor_all.Cspeed=50;
//	vTaskDelay(1000);
//	vTaskDelay(1000);
//	vTaskDelay(1000);
//	vTaskDelay(600);

	pid_mode_switch(is_No);
	motor_all.Lspeed = motor_all.Rspeed = 0;
	
	Rudder_control(170, 0); // 170 人站起来
	vTaskDelay(500);
	Rudder_control(170, 1); // 100左边 170中间 250左边
	vTaskDelay(100);
	Rudder_control(250, 1); // 100左边 170中间 250左边
	vTaskDelay(100);
	Rudder_control(100, 1); // 100左边 170中间 250左边
	vTaskDelay(100);
	Rudder_control(170, 1); // 100左边 170中间 250左边

	if(ScanMode == is_Front)
	{
		while (Infrared_ahead == 0)
		{
			vTaskDelay(5);
		} // 撞挡板
	}
	else if(ScanMode == is_Back)
	{
		while (Infrared_back == 0)
		{
			vTaskDelay(5);
		} // 撞挡板
	}
	
	if(ScanMode == is_Front)
	{
		while (Infrared_ahead == 1)
		{
			vTaskDelay(5);
		} // 撞挡板
	}
	else if(ScanMode == is_Back)
	{
		while (Infrared_back == 1)
		{
			vTaskDelay(5);
		} // 撞挡板
	}

	send_play_specified_command(30);

	// 左边 450放下 150举起
	// 右边 130放下 400举起
	Rudder_control(150, 2);
	Rudder_control(400, 4);
	vTaskDelay(100);
	Rudder_control(450, 2);
	Rudder_control(130, 4);
	
	Rudder_control(270,0);// 270 人躺下

	{
		mpuZreset(imu.yaw, nodesr.nowNode.angle);
		angle.AngleG = getAngleZ();
		motor_all.Gincrement = 0.5;
		motor_all.Gspeed = 15;
		pid_mode_switch(is_Gyro);
		float num = 0;
		num = motor_all.Distance;
		while (fabsf(motor_all.Distance - num) < 253 /*8*/)
		{
			vTaskDelay(2);
		}
		while (imu.pitch > Down_pitch)
		{
			vTaskDelay(2);
		} // 下桥
		while (imu.pitch < After_down)
		{
			vTaskDelay(2);
		} // 下桥完毕
		  // 路程记录清零
		encoder_clear();
		pid_mode_switch(is_Line);
		motor_all.Cincrement = 0.5;
		motor_all.Cspeed = 25;
	}
}

/**
 * @brief:
 * @param {float} angle1
 * @return {*}
 */
void Protect(float angle1)
{
	int num = 0;
	int breaktime = 0;
	char oriT = motor_all.GyroT_speedMax;
	struct PID_param origin_param = gyroT_pid_param;
	gyroT_pid_param.kp = 0.5;			// 0.65
	gyroT_pid_param.ki = 0.0003 /*66*/; // 0.0005
	gyroT_pid_param.kd = 0;
	motor_all.GyroT_speedMax = 10;
	buzzer_on();

	CarBrake();
	vTaskDelay(200);

	encoder_clear();
	pid_mode_switch(is_No);
	motor_all.Lspeed = motor_all.Rspeed = -8;
	while (fabsf(motor_all.Distance - num) < 5 * 60) // 10
	{
		vTaskDelay(2);
	}

	CarBrake();
	vTaskDelay(300);

	// 保护矫正
	angle.AngleT = getAngleZ() + angle1;
	pid_mode_switch(is_Turn);
	while (fabsf(angle.AngleT - getAngleZ()) > 4)
	{
		breaktime++;
		vTaskDelay(2);
		if (breaktime >= 500)
			break;
	}

	// 直走
	pid_mode_switch(is_Gyro);
	angle.AngleG = getAngleZ();
	motor_all.Gspeed = 17;

	motor_all.GyroT_speedMax = oriT;
	gyroT_pid_param = origin_param;
	breaktime = 0;
	buzzer_off();
}

void Connect(uint8_t Route[])
{
	static u8 temp = 0, i = 0;
	temp = map.point-1;
	i = 0;
	nodesr.nextNode = Node[getNextConnectNode(nodesr.nowNode.nodenum,Route[0])];
	nodesr.nowNode.angle = nodesr.nextNode.angle;
	while (1)
	{
		route[temp++] = Route[i++]; // 路线连接
		if (Route[i] == 255)
		{
			route[temp] = Route[i];
			break;
		}
	}
}

void ConnectFirstBack(void)
{
	if((Clue_Stage[2] == 8 && nodesr.nowNode.nodenum == P8)
	|| (Clue_Stage[2] == 7 && nodesr.nowNode.nodenum == P7))
	{
		if(Clue_Stage[2] == 7 && Clue_Stage[1] == 5)
		{
			if(treasure == P1 && color_flag[0] == 1)
				Connect(Clue1route);
			else if(treasure == P3 && color_flag[0] == 1)
				Connect(Clue2route);
			else if(treasure == P4 && color_flag[0] == 1)
				Connect(Clue3route);
			else if(treasure == P5 && color_flag[0] == 1)
				Connect(Clue4route);
			else if(treasure == P6 && color_flag[0] == 1)
				Connect(Clue5route);
			
			else if(treasure == P1 && color_flag[1] == 1)
				Connect(Clue6route);
			else if(treasure == P3 && color_flag[1] == 1)
				Connect(Clue7route);
			else if(treasure == P4 && color_flag[1] == 1)
				Connect(Clue8route);
			else if(treasure == P5 && color_flag[1] == 1)
				Connect(Clue9route);
			else if(treasure == P6 && color_flag[1] == 1)
				Connect(Clue10route);
			
			else if(treasure == P1 && color_flag[2] == 1)
				Connect(Clue11route);
			else if(treasure == P3 && color_flag[2] == 1)
				Connect(Clue12route);
			else if(treasure == P4 && color_flag[2] == 1)
				Connect(Clue13route);
			else if(treasure == P5 && color_flag[2] == 1)
				Connect(Clue14route);
			else if(treasure == P6 && color_flag[2] == 1)
				Connect(Clue15route);
			
			else if(treasure == P1 && color_flag[3] == 1)
				Connect(Clue16route);
			else if(treasure == P3 && color_flag[3] == 1)
				Connect(Clue17route);
			else if(treasure == P4 && color_flag[3] == 1)
				Connect(Clue18route);
			else if(treasure == P5 && color_flag[3] == 1)
				Connect(Clue19route);
			else if(treasure == P6 && color_flag[3] == 1)
				Connect(Clue20route);
		}
		else if(Clue_Stage[2] == 8 && Clue_Stage[1] == 5)
		{
			if(treasure == P1 && color_flag[0] == 1)
				Connect(Clue21route);
			else if(treasure == P3 && color_flag[0] == 1)
				Connect(Clue22route);
			else if(treasure == P4 && color_flag[0] == 1)
				Connect(Clue23route);
			else if(treasure == P5 && color_flag[0] == 1)
				Connect(Clue24route);
			else if(treasure == P6 && color_flag[0] == 1)
				Connect(Clue25route);
			
			else if(treasure == P1 && color_flag[1] == 1)
				Connect(Clue26route);
			else if(treasure == P3 && color_flag[1] == 1)
				Connect(Clue27route);
			else if(treasure == P4 && color_flag[1] == 1)
				Connect(Clue28route);
			else if(treasure == P5 && color_flag[1] == 1)
				Connect(Clue29route);
			else if(treasure == P6 && color_flag[1] == 1)
				Connect(Clue30route);
			
			else if(treasure == P1 && color_flag[2] == 1)
				Connect(Clue31route);
			else if(treasure == P3 && color_flag[2] == 1)
				Connect(Clue32route);
			else if(treasure == P4 && color_flag[2] == 1)
				Connect(Clue33route);
			else if(treasure == P5 && color_flag[2] == 1)
				Connect(Clue34route);
			else if(treasure == P6 && color_flag[2] == 1)
				Connect(Clue35route);
			
			else if(treasure == P1 && color_flag[3] == 1)
				Connect(Clue36route);
			else if(treasure == P3 && color_flag[3] == 1)
				Connect(Clue37route);
			else if(treasure == P4 && color_flag[3] == 1)
				Connect(Clue38route);
			else if(treasure == P5 && color_flag[3] == 1)
				Connect(Clue39route);
			else if(treasure == P6 && color_flag[3] == 1)
				Connect(Clue40route);
		}
		else if(Clue_Stage[2] == 7 && Clue_Stage[1] == 6)
		{
			if(treasure == P1 && color_flag[0] == 1)
				Connect(Clue41route);
			else if(treasure == P3 && color_flag[0] == 1)
				Connect(Clue42route);
			else if(treasure == P4 && color_flag[0] == 1)
				Connect(Clue43route);
			else if(treasure == P5 && color_flag[0] == 1)
				Connect(Clue44route);
			else if(treasure == P6 && color_flag[0] == 1)
				Connect(Clue45route);
			
			else if(treasure == P1 && color_flag[1] == 1)
				Connect(Clue46route);
			else if(treasure == P3 && color_flag[1] == 1)
				Connect(Clue47route);
			else if(treasure == P4 && color_flag[1] == 1)
				Connect(Clue48route);
			else if(treasure == P5 && color_flag[1] == 1)
				Connect(Clue49route);
			else if(treasure == P6 && color_flag[1] == 1)
				Connect(Clue50route);
			
			else if(treasure == P1 && color_flag[2] == 1)
				Connect(Clue51route);
			else if(treasure == P3 && color_flag[2] == 1)
				Connect(Clue52route);
			else if(treasure == P4 && color_flag[2] == 1)
				Connect(Clue53route);
			else if(treasure == P5 && color_flag[2] == 1)
				Connect(Clue54route);
			else if(treasure == P6 && color_flag[2] == 1)
				Connect(Clue55route);
			
			else if(treasure == P1 && color_flag[3] == 1)
				Connect(Clue56route);
			else if(treasure == P3 && color_flag[3] == 1)
				Connect(Clue57route);
			else if(treasure == P4 && color_flag[3] == 1)
				Connect(Clue58route);
			else if(treasure == P5 && color_flag[3] == 1)
				Connect(Clue59route);
			else if(treasure == P6 && color_flag[3] == 1)
				Connect(Clue60route);
		}
		else if(Clue_Stage[2] == 8 && Clue_Stage[1] == 6)
		{
			if(treasure == P1 && color_flag[0] == 1)
				Connect(Clue61route);
			else if(treasure == P3 && color_flag[0] == 1)
				Connect(Clue62route);
			else if(treasure == P4 && color_flag[0] == 1)
				Connect(Clue63route);
			else if(treasure == P5 && color_flag[0] == 1)
				Connect(Clue64route);
			else if(treasure == P6 && color_flag[0] == 1)
				Connect(Clue65route);
			
			else if(treasure == P1 && color_flag[1] == 1)
				Connect(Clue66route);
			else if(treasure == P3 && color_flag[1] == 1)
				Connect(Clue67route);
			else if(treasure == P4 && color_flag[1] == 1)
				Connect(Clue68route);
			else if(treasure == P5 && color_flag[1] == 1)
				Connect(Clue69route);
			else if(treasure == P6 && color_flag[1] == 1)
				Connect(Clue70route);
			
			else if(treasure == P1 && color_flag[2] == 1)
				Connect(Clue71route);
			else if(treasure == P3 && color_flag[2] == 1)
				Connect(Clue72route);
			else if(treasure == P4 && color_flag[2] == 1)
				Connect(Clue73route);
			else if(treasure == P5 && color_flag[2] == 1)
				Connect(Clue74route);
			else if(treasure == P6 && color_flag[2] == 1)
				Connect(Clue75route);
			
			else if(treasure == P1 && color_flag[3] == 1)
				Connect(Clue76route);
			else if(treasure == P3 && color_flag[3] == 1)
				Connect(Clue77route);
			else if(treasure == P4 && color_flag[3] == 1)
				Connect(Clue78route);
			else if(treasure == P5 && color_flag[3] == 1)
				Connect(Clue79route);
			else if(treasure == P6 && color_flag[3] == 1)
				Connect(Clue80route);
		}
		Clue_Num[0] = 0;
		Clue_Num[1] = 0;
		Clue_Stage[0] = 0;
		Clue_Stage[1] = 0;
		Clue_Stage[2] = 0;
	}
}


/**
 * @brief: 特殊结点
 * @return {*}
 */
void Special_Node(void)
{
	if (((nodesr.nowNode.flag & DRIGHT) == DRIGHT) & ((nodesr.nowNode.flag & CRIGHT) == CRIGHT) & (nodesr.nowNode.nodenum == N5)) // N4-N5右循迹 左循迹 边缘忽略
	{																															  // N4-N5
		nodesr.nowNode.flag &= (~RIGHT_LINE);																					  // 取消右循迹标志位
		nodesr.nowNode.flag |= LEFT_LINE;																						  // 左循迹
		while (deal_arrive() != 1)
		{
			vTaskDelay(2);
		}								  // 右分岔
		nodesr.nowNode.flag &= (~CRIGHT); // 取消右分岔标志位
		while (deal_arrive() != 1)		  // 右半边天
		{
			vTaskDelay(2);
			scaner_set.EdgeIgnore = 6;
			special_arrive = 1;
		}
	}
	else if ((nodesr.nowNode.nodenum == N4) & ((nodesr.nowNode.flag & CRIGHT) == CRIGHT)) // N5-N4
	{
		float num = 0;
		nodesr.nowNode.flag &= (~RIGHT_LINE);
		nodesr.nowNode.flag |= LEFT_LINE; // 左循迹
		while (deal_arrive() != 1)
		{
			vTaskDelay(2);
		}
		num = motor_all.Distance;
		while (fabsf(motor_all.Distance - num) < 10) // 第一个左分岔路口再走10厘米
		{
			vTaskDelay(2);
		}
		while (deal_arrive() != 1)
		{
			vTaskDelay(2);
		}
	}
	else if ((nodesr.nowNode.nodenum == N4) & ((nodesr.nowNode.flag & CLEFT) == CLEFT)) // N3-N4
	{
		float num = 0;
		nodesr.nowNode.flag &= (~LEFT_LINE);
		nodesr.nowNode.flag |= RIGHT_LINE; // 右循迹
		while (deal_arrive() != 1)
		{
			vTaskDelay(2);
		}
		num = motor_all.Distance;
		while (fabsf(motor_all.Distance - num) < 10) // 第一个左分岔路口再走10厘米
		{
			vTaskDelay(2);
		}
		while (deal_arrive() != 1)
		{
			vTaskDelay(2);
		}
	}
	else if (((nodesr.nowNode.nodenum == N13) & ((nodesr.nowNode.flag & CLEFT) == CLEFT)) & (((nodesr.nowNode.flag & CRIGHT) == CRIGHT) & ((nodesr.nowNode.flag & DLEFT) == DLEFT)) & ((nodesr.nowNode.flag & DRIGHT) == DRIGHT))
	/*P6-N13*/ {
		angle.AngleG = getAngleZ();
		motor_all.Gspeed = 300;
		pid_mode_switch(is_Gyro);
	}
	//	else if(nodesr.nowNode.nodenum==N9&(nodesr.nowNode.flag&CLEFT)==CLEFT)
	//	{
	//		float num=0;
	//		while(!deal_arrive())
	//		{
	//			vTaskDelay(2);
	//		}
	//		num=motor_all.Distance;
	//		while(motor_all.Distance-num<65)
	//		{
	//			vTaskDelay(2);
	//		}
	//		while(!deal_arrive())
	//		{
	//			vTaskDelay(2);
	//		}
	//	}
	else
	{
		angle.AngleG = nodesr.nowNode.angle;
		motor_all.Gspeed = 1000;
		pid_mode_switch(is_Gyro);
	}
	//	if(((nodesr.nowNode.flag&CRIGHT)==CRIGHT)&((nodesr.nowNode.flag&CLEFT)==CLEFT))//N5-N6  P4-N6先左循迹后右循迹
	//	{
	//		angle.AngleT=getAngleZ();
	//		pid_mode_switch(is_Gyro);
	////		nodesr.nowNode.flag|=LEFT_LINE;//左循迹
	////		while(deal_arrive()!=1)
	////		{
	////			vTaskDelay(2);
	////		}//检测到右分岔
	////		nodesr.nowNode.flag&=(~CRIGHT);//取消右分岔标志位
	////		while(deal_arrive()!=1)
	////		{
	////			vTaskDelay(2);
	////		}//检测到左分岔
	////		nodesr.nowNode.flag&=(~LEFT_LINE);//取消左循迹
	////		nodesr.nowNode.flag|=RIGHT_LINE;//右循迹
	////		special_arrive=1;
	//	}
}

void BHillChange(void)
{
	Cross_getline();
	if(Cross_Scaner.ledNum > 3 || Cross_Scaner.ledNum == 0 || Cross_Scaner.lineNum > 1)
	{
		motor_all.Gspeed = GoStage_Speed;
		angle.AngleG = getAngleZ();
		pid_mode_switch(is_Gyro);
	}
	else
	{
		motor_all.Cspeed = GoStage_Speed;
		pid_mode_switch(is_Line);
	}
}
