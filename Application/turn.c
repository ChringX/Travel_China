#include "turn.h"
#include "imu.h"
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
uint8_t Turn360_Flag = 0;
float Turn360RecallAngle = 0;
uint8_t gryo_turn=0;
uint8_t MustBeZero = 0;


/*计算两角度夹角*/
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

/*计算一次补正值*/
void mpuZreset(float sensorangle, float referangle)
{
	imu.compensateZ = need2turn(sensorangle, referangle);
}

/*返回瞬时测量值+补正值*/
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

/*平台转*/
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
		GTspeed = motor_all.GyroT_speedMax;
	else if (GTspeed <= -motor_all.GyroT_speedMax)
		GTspeed = -motor_all.GyroT_speedMax;

	if (nodesr.nowNode.nodenum == P6 || nodesr.nowNode.nodenum == P1)
	{
		motor_all.Lspeed = GTspeed;
		motor_all.Rspeed = -GTspeed * 1.1f;
	}
	else
	{
		motor_all.Lspeed = GTspeed;
		motor_all.Rspeed = -GTspeed;
	}


	return 0;
}

/*开环转圈*/
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
		Left = 2500/1.2;	Right = 2500/1.2; break;
	case P6:
		Left = 2500;		Right = 2500; break;
	case P8:   
		Left = 2500;		Right = 2500; break;
	case P3:   
		Left = 2500/1.1;	Right = 2500/1.1; break;
	case P5:   
		Left = 2500;		Right = 2500; break;
	case P4:   
		Left = 2500;		Right = 2500; break;
	case P2:   
		Left = 2500;		Right = 2500; break;
	case P1:   
		Left = 2500/1.2;	Right = 2500/1.2; break;
	default:
		Left = 2500;		Right = 2500 ; break;
	}
	FreeTurn(angle.AngleT, Left, Right);

	pid_mode_switch(is_Turn); // 进入转弯  记得close cirle
}

/*闭环转圈*/
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

/*陀螺仪原地转*/
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

/*将给的角度转为360°制*/
float Change360Angle(float Angle)
{
	if (Angle < 0)
		Angle = 360 - fabs(Angle);
	return Angle;
}

/*单步360°转*/
uint8_t Turn360Step(void)
{
	float GTspeed;
	float NowAngle;

	if(MustBeZero)
		NowAngle = 0;
	else
		NowAngle = Change360Angle(Change360Angle(imu.yaw) - Turn360RecallAngle);

	gyroT_pid.measure = 360 - NowAngle;
	gyroT_pid.target = 1;

	if (gyroT_pid.measure < 2)
	{
		motor_all.Lspeed = motor_all.Rspeed = 0;
		gyroT_pid.integral = 0;
		gyroT_pid.output = 0;
		Turn360_Flag = 0;
		return 1;
	}

	GTspeed = positional_PID(&gyroT_pid, &gyroT_pid_param);

	if (GTspeed >= motor_all.GyroT_speedMax)
		GTspeed = motor_all.GyroT_speedMax;
	else if (GTspeed <= -motor_all.GyroT_speedMax)
		GTspeed = -motor_all.GyroT_speedMax;

	motor_all.Lspeed = GTspeed;
	motor_all.Rspeed = -GTspeed;

	return 0;
}

/*原地360*/
void Turn_Angle360(void)
{
	Turn360RecallAngle = Change360Angle(imu.yaw);
	Turn360_Flag = 1;
	pid_mode_switch(is_Turn);
	while(Turn360_Flag)
	{
		MustBeZero = 1;
		vTaskDelay(500);
		MustBeZero = 0;
		while(Turn360_Flag)
			vTaskDelay(2);
	}
}

/*自平衡走*/
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
	
	// printf("%.2f %.2f %.2f\r\n", gyroG_pid.measure,speed,GGspeed);

	motor_all.Lspeed = speed + GGspeed;
	motor_all.Rspeed = speed - GGspeed;

	// motor_all.Lspeed = speed - GGspeed;
	// motor_all.Rspeed = speed + GGspeed;

	return 1;
}

/*开环转*/
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


