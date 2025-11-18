/*
 * @File: turn.c
 * @Description: 
 * @Version: 1.0.0
 * @Author: 
 * @Date: 2023-09-13 20:33:34
 * @LastEditTime: 2023-09-15 15:16:33
 */
#include "turn.h"
#include "imu_task.h"
#include "pid.h"
#include "speed_ctrl.h"
#include "math.h"
#include "map.h"
#include "motor_task.h"
#include "uart.h"
#include "motor.h"
#include "scaner.h"
#include "openmv.h"
#include "encoder.h"
#include "motor_task.h"
#include "bsp_buzzer.h"
volatile struct Angle angle = {0, 0};
static uint8_t Turn360_Flag = 0;
uint8_t gryo_turn=0;
// 输出当前角度与目标角度的最小夹角

/**
 * @brief: 
 * @param {float} nowangle
 * @param {float} targetangle
 * @return {*}
 */
float need2turn(float nowangle, float targetangle)
{
	float need2Turn;
	need2Turn = targetangle - nowangle; // 实际所需转的角度
	if (need2Turn > 180)
	{
		need2Turn -= 360;
	}
	else if (need2Turn <= -180)
	{
		need2Turn += 360;
	}

	return need2Turn;
}

float Modified_Angle(float angle,float angle_add)
{
	angle = angle_add + 180;
	if (angle > 180)
	{
		angle -= 360;
	}
	else if (angle <= -180)
	{
		angle += 360;
	}
	return angle;
}

//换头并以头为基准设imu
void Change_MODE(void)
{
	//如果要换头-->is turn
	if(ScanMode == is_Front)
	{
		MODE_Switch(is_Back);
	}else
	{
		MODE_Switch(is_Front);
	}
	
	nodesr.nowNode.angle += 180;
	if(nodesr.nowNode.angle > 180)
	{
		nodesr.nowNode.angle -= 360;
	}
	mpuZreset(imu.yaw,nodesr.nowNode.angle);
}

/**
 * @brief: 陀螺仪软校准
 * @param {float} sensorangle
 * @param {float} referangle
 * @return {*}
 */
void mpuZreset(float sensorangle, float referangle)
{
	imu.compensateZ = need2turn(sensorangle, referangle);
}

/**
 * @brief: 获取Z角度
 * @return {*}
 */
float getAngleZ(void)
{
	float targetangle;
	targetangle = imu.yaw + imu.compensateZ;

	if (targetangle > 180)
		targetangle -= 360;
	else if (targetangle <= -180)
		targetangle += 360;

	return targetangle;
}


/**
 * @brief: 平台转向
 * @param {float} Angle
 * @return {*}
 * @note  真闭环 双环  记得close cirle
 */
uint8_t Stage_turn_Angle(float Angle)
{
	float GTspeed;
	float now_angle;

	now_angle = getAngleZ();
	if (fabsf(Angle - now_angle) < 2)
	{
		motor_all.Lspeed = motor_all.Rspeed = 0;
		gyroT_pid.integral = 0;
		gyroT_pid.output = 0;
		return 1;
	}

	gyroT_pid.measure = need2turn(now_angle, Angle);
	gyroT_pid.target = 0;

	GTspeed = positional_PID(&gyroT_pid, &gyroT_pid_param);

	if (GTspeed >= motor_all.GyroT_speedMax)
	{
		GTspeed = motor_all.GyroT_speedMax;
		// bofang_zhiding(13);
	}
	else if (GTspeed <= -motor_all.GyroT_speedMax)
		GTspeed = -motor_all.GyroT_speedMax;

	motor_all.Lspeed = GTspeed;
	motor_all.Rspeed = -GTspeed;

	return 0;
}

/**
 * @brief: 陀螺仪Z轴转角度
 * @param {float} Angle1 要转的角度
 * @return {*}
 */
void Turn_Angle_Relative_Open(float Angle1) // 左180到右-180,速度必须是正的，
{
	float Turn_Angle_Before = 0, Turn_Angle_Targe = 0;
	float Left = 0;
	float Right = 0;
	Turn_Angle_Before = getAngleZ();			   // 读取当前的角度
	Turn_Angle_Targe = Turn_Angle_Before + Angle1; // 目标角度设为绝对坐标
	/*******************如果存在临界状态，把目标角度转化为绝对坐标******180 0 -180*************/
	if (Turn_Angle_Targe > 180)
	{
		Turn_Angle_Targe = Turn_Angle_Targe - 360;
	}
	else if (Turn_Angle_Targe < -180)
	{
		Turn_Angle_Targe = Turn_Angle_Targe + 360;
	}

	angle.AngleT = Turn_Angle_Targe;

	switch (nodesr.nowNode.nodenum)
	{
	case P7:
		Left = 2500;    	Right = 2500; break;
	case P6:
		Left = 2500;  	  Right = 2500;	break;
	case P8:   
		Left = 2500;	  Right = 2500; break;
	case P3:   
		Left = 2500;   	 Right = 2500; break;
	case P5:   
		Left = 2500; 	 Right = 2500; break;
	case P4:   
		Left = 2500;     Right = 2500; break;
	case P2:   
		Left = 2500;	 Right = 2500; break;
	case P1:   
		Left = 2500;	 Right = 2500; break;
	default:
		Left = 2500;     Right = 2500 ; break;
	}
	FreeTurn(angle.AngleT, Left, Right);

	pid_mode_switch(is_Turn); // 进入转弯  记得close cirle
}

/**
 * @brief: 
 * @param {float} Angle1
 * @return {*}
 */
void Turn_Angle_Relative(float Angle1) // 左180到右-180,速度必须是正的，
{
	float Turn_Angle_Before = 0, Turn_Angle_Targe = 0;

	Turn_Angle_Before = getAngleZ();			   // 读取当前的角度//@@@@@
	Turn_Angle_Targe = Turn_Angle_Before + Angle1; // 目标角度设为绝对坐标
	/*******************如果存在临界状态，把目标角度转化为绝对坐标******180 0 -180*************/
	if (Turn_Angle_Targe > 180)
	{
		Turn_Angle_Targe = Turn_Angle_Targe - 360;
	}
	else if (Turn_Angle_Targe < -180)
	{
		Turn_Angle_Targe = Turn_Angle_Targe + 360;
	}

	angle.AngleT = Turn_Angle_Targe;
	pid_mode_switch(is_Turn); // 进入转弯
}

/**
 * @brief: 陀螺仪Z轴结合PID原地转角度
 * @param {float} Angle 填要转到的角度(绝对角度)
 * @return {*}
 * @note  用法：Target = ?,PIDMode = is_Turn,while(fabs(getAngleZ()-Target)<1);
 * 		  记得close闭环
 */
uint8_t Turn_Angle(float Angle)
{
	float GTspeed;
	float now_angle;
	
	if (Angle > 180)
	{
		Angle -= 360;
	}
	else if (Angle < -180)
	{
		Angle += 360;
	}

	now_angle = getAngleZ();

	if (fabsf(Angle - now_angle) < 2)
	{
		motor_all.Lspeed = motor_all.Rspeed = 0;
		gyroT_pid.integral = 0;
		gyroT_pid.output = 0;
		return 1;
	}

	gyroT_pid.measure = need2turn(now_angle, Angle);
	gyroT_pid.target = 0;

	GTspeed = positional_PID(&gyroT_pid, &gyroT_pid_param);

	if (GTspeed >= motor_all.GyroT_speedMax)
	{
		GTspeed = motor_all.GyroT_speedMax;
	}
	else if (GTspeed <= -motor_all.GyroT_speedMax)
	{
		GTspeed = -motor_all.GyroT_speedMax;
	}

	motor_all.Lspeed = GTspeed;
	motor_all.Rspeed = -GTspeed;
	return 0;
}

/**
 * @brief: 原地旋转360度
 * @return {*}
 * @note   运用多次旋转，设定阈值，到达目标值前进行更新
 */
void Turn_Angle360(void)
{
	Turn360_Flag = 1;
	Turn_Angle_Relative_Open(179);
	Turn360_Flag = 0;
	Turn_Angle_Relative_Open(179);
	while (fabs(angle.AngleT - getAngleZ()) > 4) // 4
	{
		vTaskDelay(2);
	}

}

/**
 * @brief: 陀螺仪Z轴自平衡行走 陀螺仪单环
 * @param {float} angle_want
 * @param {float} speed
 * @return {*}
 */
uint8_t runWithAngle(float angle_want, float speed)
{
	float GGspeed, now_angle;
	
	now_angle = getAngleZ();

	gyroG_pid.measure = need2turn(now_angle, angle_want);
	gyroG_pid.target = 0;

	GGspeed = positional_PID(&gyroG_pid, &gyroG_pid_param);

	GGspeed = GGspeed * speed / 50;

	if (GGspeed >= motor_all.GyroG_speedMax)
	{
		GGspeed = motor_all.GyroG_speedMax;
	}
	else if (GGspeed <= -motor_all.GyroG_speedMax)
	{
		GGspeed = -motor_all.GyroG_speedMax;
	}
	
	motor_all.Lspeed = speed + GGspeed;
	motor_all.Rspeed = speed - GGspeed;

	return 1;
}

/**
 * @brief: 双环矫正
 * @param {float} Angle
 * @return {*}
 * @note   转两个绝对角度之间的差值，一个是实时角度，一个是想要的绝对角度
 */
uint8_t Turn_Fix(float Angle)
{
	float GTspeed;
	float now_angle;
	char origin_max = motor_all.GyroT_speedMax;
	motor_all.GyroT_speedMax = 25; // 20//24

	if (Angle > 180)
	{
		Angle -= 360;
	}
	else if (Angle < -180)
	{
		Angle += 360;
	}

	now_angle = getAngleZ();

	if (fabsf(Angle - now_angle) < 2)
	{
		motor_all.Lspeed = motor_all.Rspeed = 0;
		gyroT_pid.integral = 0;
		gyroT_pid.output = 0;
		return 1;
	}

	gyroT_pid.measure = need2turn(now_angle, Angle);
	gyroT_pid.target = 0;

	GTspeed = positional_PID(&gyroT_pid, &gyroT_pid_param);

	if (GTspeed >= motor_all.GyroT_speedMax)
	{
		GTspeed = motor_all.GyroT_speedMax;
	}
	else if (GTspeed <= -motor_all.GyroT_speedMax)
	{
		GTspeed = -motor_all.GyroT_speedMax;
	}
		
	motor_all.Lspeed = GTspeed;
	motor_all.Rspeed = -GTspeed;

	motor_all.GyroT_speedMax = origin_max;
	return 0;
}

/**
 * @brief: 差速转（最好还是用上pid吧）
 * @param {float} speed
 * @param {float} radius
 * @return {*}
 */
void AdCircle(float speed, float radius)
{
	motor_all.Lspeed = speed - radius;
	motor_all.Rspeed = speed + radius;
}

/**
 * @brief: 
 * @param {float} speed
 * @param {float} Angle
 * @return {*}
 */
uint8_t Drift(float speed, float Angle)
{
	float GPspeed, now_angle;

	// 临界处理
	if (Angle > 180)
	{
		Angle -= 360;
	}
	else if (Angle < -180)
	{
		Angle += 360;
	}
		
	now_angle = getAngleZ();
	if (fabsf(Angle - now_angle) < 1)
	{
		motor_all.Lspeed = motor_all.Rspeed = speed;
		GyroP_pid.integral = 0;
		GyroP_pid.output = 0;
		return 1;
	}
	GyroP_pid.measure = need2turn(now_angle, Angle);
	GyroP_pid.target = 0;

	GPspeed = positional_PID(&GyroP_pid, &GyroP_pid_param);
	motor_all.GyroP_speedMax = speed * 0.45f;
	if (GPspeed >= motor_all.GyroP_speedMax)
	{
		GPspeed = motor_all.GyroP_speedMax;
	}
		
	else if (GPspeed <= -motor_all.GyroP_speedMax)
	{
		GPspeed = -motor_all.GyroP_speedMax;
	}
	//    if(GPspeed<0)
	//	{
	//		motor_all.Lspeed = speed+GPspeed*speed/50;
	//		motor_all.Rspeed = speed-GPspeed*speed*1.3f/50;
	//	}
	//	else{
	motor_all.Lspeed = speed + GPspeed * speed / 50;
	motor_all.Rspeed = speed - GPspeed * speed / 50;
	//	}

	return 0;
}

/**
 * @brief: 
 * @param {int} Cspeed
 * @return {*}
 */
float Get_F(int Cspeed)
{
	float C = 0;
	C = (float)(Speed[0] + Speed[1]) / (Speed[2] + Speed[3]);
	float f = 0;
	f = ((C - 1) * Cspeed / (C + 1)) - Fspeed;
	return f;
}

/**
 * @brief:
 * @param {float} Angle
 * @param {float} L
 * @param {float} R
 * @return {*}
 */
void FreeTurn(float Angle, float L, float R)
{
	pid_mode_switch(is_Free);
	if (Turn360_Flag == 1)
	{
		motor_set_pwm(1, -L);
		motor_set_pwm(2, -L);
		motor_set_pwm(3, R);
		motor_set_pwm(4, R);
		while (fabsf(Angle - getAngleZ()) > 5) // 如果角度相差12一直转
		{
			vTaskDelay(2);
		}
	}
	else
	{
		if (need2turn(getAngleZ(), Angle) > 0) // 逆时针转
		{
			while (fabsf(Angle - getAngleZ()) > 20) // 如果角度相差12一直转//20//30
			{
				motor_set_pwm(1, -L);
				motor_set_pwm(2, -L);
				motor_set_pwm(3, R);
				motor_set_pwm(4, R);
				if (nodesr.nowNode.function != UpStage && nodesr.nowNode.function != BSoutPole && nodesr.nowNode.function != BHM)
				{
					getline_error();
					if (Scaner.lineNum == 1 && ((Scaner.detail & 0x7E0) != 0) && (fabs(need2turn(nodesr.nextNode.angle, getAngleZ())) < fabs(need2turn(nodesr.nextNode.angle, nodesr.nowNode.angle)) * 0.3f))
					{
						break;
					}
				}
			}
			//		}
		}
		if (need2turn(getAngleZ(), Angle) < 0) // 顺时针转
		{
			while (fabsf(Angle - getAngleZ()) > 20) // 如果角度相差12一直转
			{
				motor_set_pwm(1, L);
				motor_set_pwm(2, L);
				motor_set_pwm(3, -R);
				motor_set_pwm(4, -R);
				if (nodesr.nowNode.function != UpStage && nodesr.nowNode.function != BSoutPole && nodesr.nowNode.function != BHM)
				{
					getline_error();
					if (Scaner.lineNum == 1 && ((Scaner.detail & 0x7E0) != 0) && (fabs(need2turn(nodesr.nextNode.angle, getAngleZ())) < fabs(need2turn(nodesr.nextNode.angle, nodesr.nowNode.angle)) * 0.3f))
					{
						break;
					}
				}
			}
		}
	}
}


