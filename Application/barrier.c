#include "barrier.h"
#include "sys.h"
#include "delay.h"
#include "speed_ctrl.h"
#include "motor.h"
#include "pid.h"
#include "imu.h"
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
#define IMPACT_SPEED 16
#define QQB_Speed 7-1
#define LiuShuiRate_Default	1.6				//默认流水倍率
#define LiuShuiRate_UM		1.82			//珠峰流水倍率
#define LiuShuiRate_USP		1.65			//南极流水倍率 //1.3 //1.65
#define LiuShuiRate_UB		1.6				//长桥流水倍率
#define LiuShuiRate_ST		2.0				//平台流水倍率
#define LiuShuiRate_BG		1.8				//出发流水倍率

uint8_t WavePlateLeft_Flag = 0;
uint8_t WavePlateRight_Flag = 0;
uint8_t color_flag[5] = {0, 0, 0, 0, 0}; // 0:D2、1:D3、2:D4、3:D5、4:D1
uint8_t isStage = 0;
uint8_t treasure[3] = {0};				 // 宝藏
uint8_t value;							 // openmv接口
uint8_t DownLiuShui = 0;				 // 流水下坡标志位
float 	LiuShuiRate = LiuShuiRate_Default; // 流水下坡前轮速度倍率
uint8_t special_arrive = 0;



/*平台 - 不包括P2*/
void Stage(void)
{
	/*参数调整*/
	isStage = 1;
	struct PID_param origin_param1 = gyroG_pid_param;
	struct PID_param origin_param2 = line_pid_param;
	gyroG_pid_param.kp = 2;	//4.5
	gyroG_pid_param.ki = 0;
	gyroG_pid_param.kd = 30;
	line_pid_param.kp = 12*2;
	line_pid_param.ki = 0;
	line_pid_param.kd = 400;

	/*设置起始模式速度*/
	encoder_clear();
	Motor_Control(is_Line,UpStage_Speed-5,UpStage_Speed-5,0);

	scaner_set.EdgeIgnore = 6;
	Cross_getline();
	if ((Cross_Scaner.detail & 0x180) == 0x180)
		mpuZreset(imu.yaw, getAngleZ());
	
	uint8_t breakflag = 0;
	while (imu.pitch < (basic_p + 8))
	{
		
		vTaskDelay(2);
	}
		
	
	while (1)
	{
		Cross_getline();

		if (Cross_Scaner.ledNum > 5 || Cross_Scaner.lineNum >= 2)
		{
			breakflag++;
			if (breakflag >= 5)	//3
				break;
		}
		vTaskDelay(2);
	}
	scaner_set.EdgeIgnore = 0;

	buzzer_on();
	/*设置自平衡速度*/
	Robot_Work(BODY, UP); 	// 人站起来

	/*设置自平衡速度*/
	Motor_Control(is_Gyro,GoStage_Speed-2,GoStage_Speed-2,getAngleZ());
	encoder_clear();
	
	/*检测挡板*/
	while (Infrared_ahead == 0)
		vTaskDelay(5);
	
	buzzer_off();
	
	motor_all.Gspeed = IMPACT_SPEED;
	Want2Go(10);
	encoder_clear();
	open_K210();
	CarBrake();
	vTaskDelay(250);

	/*获取下一宝藏点*/
	if(treasure[0] == 0 || treasure[1] == 0 || treasure[2] == 0)
		WaitForK210();

	mpuZreset(imu.yaw, nodesr.nowNode.angle); 		//陀螺仪校正

	/*后退一段距离*/
	Motor_Control(is_Free,-1500,-1500,0);
	Want2Go(7.5);
	encoder_clear();
	CarBrake();
	vTaskDelay(100);

	/*平台动作*/
	switch (nodesr.nowNode.nodenum)
	{
	case P1:	//2
		send_play_specified_command(5);
		break;
	case P3:	//3
		send_play_specified_command(4);
		break;
	case P4:	//4
		send_play_specified_command(3);
		break;
	case P5:	//6
		send_play_specified_command(1);
		break;
	case P6:	//5
		send_play_specified_command(2);
		break;
	default:
		break;
	}
	Arrived_Stage();

	/*转身*/
	Turn_Angle_Relative(179);
	while (fabs(angle.AngleT - getAngleZ()) > 2)
		vTaskDelay(2);

	CarBrake();
	vTaskDelay(100);

	/*发现宝藏流程*/
	if ((nodesr.nowNode.nodenum == P3 && treasure[0] == 3) ||
		(nodesr.nowNode.nodenum == P4 && treasure[0] == 4) ||
		(nodesr.nowNode.nodenum == P5 && treasure[1] == 6) ||
		(nodesr.nowNode.nodenum == P6 && treasure[1] == 5))
	{
		motor_pid_clear();
		Robot_Work(LARM, UP);
		Robot_Work(RARM, UP);
		send_play_specified_command(9);
		Turn_Angle360();
		Robot_Work(LARM, DOWN);
		Robot_Work(RARM, DOWN);
	}

	motor_pid_clear();
	
	/*下平台*/
	Motor_Control(is_Line, Rubbish_Speed, Rubbish_Speed, 0);
	while (imu.pitch > basic_p - 15)
		vTaskDelay(2);
	gyroG_pid_param = origin_param1;
	LiuShuiRate = LiuShuiRate_ST;
	DownLiuShui = 1;
	while (imu.pitch < After_down)
		vTaskDelay(2);
	LiuShuiRate = LiuShuiRate_Default;
	DownLiuShui = 0;
	/*下完平台*/

	encoder_clear();
	Motor_Control(is_Line, Rubbish_Speed, Rubbish_Speed, 0);
	Want2Go(10);
	Robot_Work(BODY,DOWN);		 		// 人躺下
	line_pid_param = origin_param2;
	nodesr.nowNode.function = 0;		// 清除障碍标志
	nodesr.flag |= 0x04;		 		// 到达路口
}

/*平台 - P2*/
void Stage_P2()
{
	/*参数调整*/
	isStage = 1;
	static uint8_t Backtimes = 0; // 回来次数 - 为1时代表第二轮回家
	struct PID_param origin_param1 = gyroG_pid_param;

	/*设置起始模式速度*/
	encoder_clear();
	Motor_Control(is_Line, UpStage_Speed, UpStage_Speed, 0);

	/*寻找合适的上坡角度*/
	float tempAngle = -1;
	scaner_set.EdgeIgnore = 4;
	while (Scaner.ledNum < 8)
	{
		Cross_getline();
		if(((Cross_Scaner.detail & 0x180) == 0x180) && Cross_Scaner.ledNum < 5)
			tempAngle = getAngleZ();
		vTaskDelay(2);
	}
	if(tempAngle == -1)
		tempAngle = getAngleZ();

	/*设置自平衡速度*/
	Motor_Control(is_Gyro, GoStage_Speed, GoStage_Speed, tempAngle);
	Robot_Work(BODY, UP);														// 人站起来

	/*到平台上*/
	encoder_clear();
	Want2Go(45);

	/*刹车*/
	CarBrake();
	vTaskDelay(200);

	/*若为第二轮回家*/
	if(Backtimes == 1)
		while(1);
	
	/*转身*/
	Turn_Angle_Relative(179);
	while (fabsf(need2turn(angle.AngleT, getAngleZ())) > 2) // 7
		vTaskDelay(2);

	Backtimes++;
	gyroG_pid_param = origin_param1;
	encoder_clear();
	motor_all.Cspeed = 0;
	motor_pid_clear();
	nodesr.nowNode.function = 0; // 清除障碍标志
	nodesr.flag |= 0x04;		 // 到达路口
}

/*长桥*/
void Barrier_Bridge(void)
{
	float num = 0;
	float now_angle = 0;
	struct PID_param origin_param1 = gyroG_pid_param;
	gyroG_pid_param.kp = 1.25;
	gyroG_pid_param.ki = 0.004;
	gyroG_pid_param.kd = 0;
	
	Motor_Control(is_Line, GoStage_Speed, GoStage_Speed, 0);
	encoder_clear();
	num = motor_all.Distance;
	while (fabsf(motor_all.Distance - num) < 25)
	{
		if (Scaner.ledNum >= 3 || Scaner.lineNum >= 2 || Scaner.lineNum == 0 || Scaner.ledNum == 0)
			break;
		vTaskDelay(2);
	}

	/*准备上桥*/
	encoder_clear();
	Motor_Control(is_Gyro, GoStage_Speed, GoStage_Speed, getAngleZ());
	while (imu.pitch < Up_pitch)
		vTaskDelay(2);
	
	/*上桥中*/
	infrare_open = 1;
	now_angle = angle.AngleG; 
	while (imu.pitch > After_up)
	{
		vTaskDelay(2);
		getline_error();

		if ((infrared.head_left == 0 && infrared.head_right == 1) || (Scaner.detail & 0X000F))
			angle.AngleG = now_angle + 2.7f;
		else if ((infrared.head_left == 1 && infrared.head_right == 0) || (Scaner.detail & 0XF000))
			angle.AngleG = now_angle - 2.7f;
	}
	num = motor_all.Distance;
	buzzer_on();
	while (fabsf(motor_all.Distance - num) < 5)
	{
		vTaskDelay(2);
		getline_error();

		if ((infrared.head_left == 0 && infrared.head_right == 1) || (Scaner.detail & 0X000F))
			angle.AngleG = now_angle + 2.7f;
		else if ((infrared.head_left == 1 && infrared.head_right == 0) || (Scaner.detail & 0XF000))
			angle.AngleG = now_angle - 2.7f;
	}
	encoder_clear();

	/*到桥中*/
	num = motor_all.Distance;
	motor_all.Gspeed = Bridge_Speed;
	while (imu.pitch > basic_p - 5)
	{
		vTaskDelay(2);
		getline_error();

		if ((infrared.head_left == 0 && infrared.head_right == 1) || (Scaner.detail & 0X000F))
			angle.AngleG = now_angle + 2.5f;
		else if ((infrared.head_left == 1 && infrared.head_right == 0) || (Scaner.detail & 0XF000))
			angle.AngleG = now_angle - 2.5f;

		if (fabsf(motor_all.Distance - num) > 80-10)
			motor_all.Gspeed = Rubbish_Speed;
		if (fabsf(motor_all.Distance - num) > 116) 
			break;
	}
	buzzer_off();

	while(imu.pitch > basic_p - 15)
		vTaskDelay(2);
	/*检测到在下坡*/
	gyroG_pid_param = origin_param1;
	LiuShuiRate = LiuShuiRate_UB;
	DownLiuShui = 1;
	while(imu.pitch <= basic_p - 15)
		vTaskDelay(2);
	DownLiuShui = 0;
	LiuShuiRate = LiuShuiRate_Default;
	origin_param1 = gyroG_pid_param;
	gyroG_pid_param.kp = 0.5;
	gyroG_pid_param.ki = 0;
	gyroG_pid_param.kd = 0.5;
	while(imu.pitch < After_down)
		vTaskDelay(2);

	infrare_open = 0;
	Motor_Control(is_Line, Rubbish_Speed, Rubbish_Speed, 0);
	gyroG_pid_param = origin_param1;
	motor_all.Distance = 0;
	motor_all.encoder_avg = 0;
	nodesr.nowNode.function = 0;
	nodesr.flag |= 0X04; // 到达路口
}

/*楼梯*/
void Barrier_Hill(void)
{
	struct PID_param origin_param1 = line_pid_param;
	motor_all.Cspeed = Gyro_Speed;
	infrare_open = 1;
	uint16_t break_times = 0;
	while (infrared.head_left == 1 && infrared.head_right == 1)
	{
		break_times++;
		if(break_times > 1000)
			break;
		Cross_getline();
		if ((Cross_Scaner.detail & 0X180) == 0X180)
			mpuZreset(imu.yaw, nodesr.nowNode.angle);
		vTaskDelay(2);
	}

	line_pid_param.kp = 20;
	line_pid_param.ki = 0.004;
	line_pid_param.kd = 0;
	scaner_set.EdgeIgnore = 3;

	/*平地*/
	while (imu.pitch < basic_p + 8)
	{
		CGChange(GoStage_Speed);
		vTaskDelay(2);
	}
	/*平地->上平台*/
	buzzer_on();
	while (imu.pitch > basic_p - 8)
	{
		CGChange(GoStage_Speed);
		vTaskDelay(2);
	}
	/*下平台*/
	while (imu.pitch < basic_p - 5)
	{
		CGChange(GoStage_Speed);
		vTaskDelay(2);
	}
	/*下完平台到平地*/
	buzzer_off();
	scaner_set.EdgeIgnore = 0;
	Motor_Control(is_Line, Rubbish_Speed, Rubbish_Speed, 0);
	Want2Go(10);					// 往前走一点防止弹射起步
	encoder_clear();
	nodesr.nowNode.function = 0; 	// 清除障碍标志
	line_pid_param = origin_param1;
	nodesr.flag |= 0x04;		 	// 到达路口
}

/*刀山*/
void Sword_Mountain(void)
{
	float num;
	uint8_t getZ = 0;
	struct PID_param origin_param = line_pid_param;
	struct PID_param origin_param1 = gyroG_pid_param;
	motor_all.Cspeed = Gyro_Speed - 3;

	line_pid_param.kp = 20;
	line_pid_param.ki = 0;
	line_pid_param.kd = 350;
	
	encoder_clear();
	Want2Go(5);
	mpuZreset(imu.yaw, nodesr.nowNode.angle);

	Cross_getline();
	while (Cross_Scaner.ledNum <= 3)
	{
		Cross_getline();
		if((Cross_Scaner.detail & 0X180) == 0X180)
		{
			angle.AngleG = getAngleZ();
			getZ = 1;
		}
		vTaskDelay(2);
	}
	if(getZ == 0)
		angle.AngleG = nodesr.nowNode.angle;
	
	buzzer_on();
	motor_all.Gspeed = Gyro_Speed - 3;
	pid_mode_switch(is_Gyro);
	num = motor_all.Distance;
	while (imu.pitch < After_up) 		// 出循环上刀山
	{
		vTaskDelay(2);
		if (fabsf(motor_all.Distance - num) > 30)
			break;
	}

	encoder_clear();
	num = motor_all.Distance;
	while (imu.pitch > After_down+2) 	// 出循环下刀山
	{
		vTaskDelay(2);
		if (fabsf(motor_all.Distance - num) > 80)
			break;
	}

	buzzer_off();
	line_pid_param = origin_param;
	gyroG_pid_param = origin_param1;
	nodesr.nowNode.function = 0; 		// 清除障碍标志
	nodesr.flag |= 0x04;		 		// 到达路口
}

/*上珠峰 - 已接下珠峰*/
void Barrier_HighMountain(float speed)
{
	float num = 0;
	uint8_t getZ = 0; 
	struct PID_param origin_param1 = gyroG_pid_param;
	float origin_turnM = motor_all.GyroT_speedMax;
	motor_all.GyroT_speedMax = 25;
	motor_all.Cspeed = Mount_Speed;
	gyroG_pid_param.kp = 4.5;
	gyroG_pid_param.ki = 0;
	gyroG_pid_param.kd = 0;

	/*上坡前*/
	while ((Scaner.ledNum < 3 && Scaner.ledNum > 0) || Scaner.lineNum == 1)
	{
		if ((Scaner.detail & 0X180) == 0X180)
			mpuZreset(imu.yaw, nodesr.nowNode.angle);		//走直了就矫正
	}
	
	/*上坡*/
	Motor_Control(is_Gyro, Mount_Speed, Mount_Speed, nodesr.nowNode.angle);
	encoder_clear();
	while(imu.pitch < Up_pitch)
		vTaskDelay(2);
	buzzer_on();
	Motor_Control(is_Line, Mount_Speed, Mount_Speed, 0);
	encoder_clear();
	num = motor_all.Distance;
	while(fabsf(motor_all.Distance - num) < 90)	/*80*/
	{
		vTaskDelay(2);
		Cross_getline();
		if((Scaner.detail & 0X180) == 0X180)
		{
			angle.AngleG = getAngleZ();
			getZ = 1;
		}
		if(Cross_Scaner.ledNum >= 6)
			break;
	}
	buzzer_off();
	if(getZ == 0)
		angle.AngleG = nodesr.nowNode.angle;
	getZ = 0;
	Motor_Control(is_Gyro, Mount_Speed-5, Mount_Speed-5, angle.AngleG);
	while (imu.pitch > After_up)
		vTaskDelay(2);
	/*上完坡到平台*/
	do
	{
		vTaskDelay(2);
		getline_error();
	} while (Scaner.ledNum <= 4);
	do
	{
		vTaskDelay(2);
		getline_error();
	} while (Scaner.ledNum >= 4);
	// while (imu.pitch < Up_pitch)
	// 	vTaskDelay(2);

	/*上坡*/
	buzzer_on();
	Robot_Work(BODY, UP);
	scaner_set.EdgeIgnore = 3;
	Motor_Control(is_Line, Mount_Speed, Mount_Speed, 0);
	encoder_clear();
	num = motor_all.Distance;
	Want2Go(20);
	while (Scaner.ledNum < 10)
	{
		vTaskDelay(2);
		if((Scaner.detail & 0X180) == 0X180)
		{
			angle.AngleG = getAngleZ();
			getZ = 1;
		}
	}
	if(getZ == 0)
		angle.AngleG = nodesr.nowNode.angle;
	scaner_set.EdgeIgnore = 0;
	buzzer_off();

	/*上完坡,撞挡板*/
	Motor_Control(is_Gyro, 10, 10, angle.AngleG);
	while (Infrared_ahead == 0)
		vTaskDelay(5);
	encoder_clear();
	Want2Go(10);
	CarBrake();
	vTaskDelay(250);
	mpuZreset(imu.yaw, nodesr.nowNode.angle); // 陀螺仪校正
	Arrived_Stage();

	/*后退*/
	encoder_clear();
	Motor_Control(is_Free,-2000,-2000,0);
	Want2Go(6);
	CarBrake();
	vTaskDelay(100);

	send_play_specified_command(14);

	/*转180°*/
	Turn_Angle_Relative(179);
	while (fabs(angle.AngleT - getAngleZ()) > 2)
		vTaskDelay(2);

	/*宝藏*/
	if (treasure[2] == 8)
	{
		Robot_Work(LARM, UP);
		Robot_Work(RARM, UP);
		send_play_specified_command(9);
		Turn_Angle360();
		Robot_Work(LARM, DOWN);
		Robot_Work(RARM, DOWN);
	}

	Barrier_Down_HighMountain(666.666);
	gyroG_pid_param = origin_param1;
	motor_all.GyroT_speedMax = origin_turnM;
	nodesr.nowNode.function = 0; // 清除障碍标志
	nodesr.flag |= 0x04;		 // 到达路口
}

/*下珠峰*/
void Barrier_Down_HighMountain(float speed)
{
	/*掉头之后陀螺仪自平衡*/
	Motor_Control(is_Gyro, Old_M_Speed, Old_M_Speed, getAngleZ());
	Robot_Work(BODY, DOWN);

	/*扫红线判断是否下坡*/
	do
	{
		vTaskDelay(2);
		getline_error();
	} while (Scaner.ledNum <= 4);
	do
	{
		vTaskDelay(2);
		getline_error();
	} while (Scaner.ledNum >= 4);

	/*前进一小段*/
	encoder_clear();
	struct PID_param origin_param = line_pid_param;
	line_pid_param.kp = 35;
	line_pid_param.ki = 0;
	line_pid_param.kd = 1.5;
	Motor_Control(is_Line, Old_M_Speed, Old_M_Speed, 0);
	Want2Go(35);
	
	
	/*切换循迹，开始下坡*/
	line_pid_param.kp = 12*2;
	line_pid_param.ki = 0;
	line_pid_param.kd = 400;
	Motor_Control(is_Line, Rubbish_Speed, Rubbish_Speed, 0);
	uint8_t getZ1 = 0;
	LiuShuiRate = LiuShuiRate_UM;
	while(Scaner.ledNum < 4)
	{
		getline_error();
		if(Scaner.detail == 0x180 && !getZ1)
		{
			getZ1 = 1;
			angle.AngleG = getAngleZ();
		}
		vTaskDelay(2);
	}
	if(!getZ1)
		angle.AngleG = 180;
	LiuShuiRate = LiuShuiRate_Default;
	Motor_Control(is_Gyro, Rubbish_Speed, Rubbish_Speed, angle.AngleG);
	while (imu.pitch < After_down)
		vTaskDelay(2);
	encoder_clear();

	/*下第二个坡*/
	do
	{
		vTaskDelay(2);
		getline_error();
	} while (Scaner.ledNum <= 4);
	motor_all.Gspeed = Rubbish_Speed;
	do
	{
		vTaskDelay(2);
		getline_error();
	} while (Scaner.ledNum >= 4);
	motor_all.Gspeed = Old_M_Speed;
	/*前进一小段*/
	encoder_clear();
	Want2Go(5);
	line_pid_param.kp = 35;
	line_pid_param.ki = 0;
	line_pid_param.kd = 1.5;
	Motor_Control(is_Line, Old_M_Speed, Old_M_Speed, 0);
	Want2Go(35);

	/*切换循迹，开始下坡*/
	line_pid_param.kp = 12;
	line_pid_param.ki = 0;
	line_pid_param.kd = 400;
	Motor_Control(is_Line, Rubbish_Speed+5, Rubbish_Speed+5, 0);
	uint8_t getZ2 = 0;
	LiuShuiRate = LiuShuiRate_UM;
	encoder_clear();
	float num = motor_all.Distance;
	while (Scaner.ledNum < 4 && Scaner.ledNum > 0)
	{
		getline_error();
		if (Scaner.detail == 0x180 && !getZ2)
		{
			getZ2 = 1;
			angle.AngleG = getAngleZ();
		}
		if(fabsf(motor_all.Distance - num) > 60)
			break;
		vTaskDelay(2);
	}
	if (!getZ2)
		angle.AngleG = 180;
	LiuShuiRate = LiuShuiRate_Default;
	Motor_Control(is_Gyro, Rubbish_Speed+5+5, Rubbish_Speed+5+5, angle.AngleG);
	while (imu.pitch < After_down)
		vTaskDelay(2);
	Motor_Control(is_Line, Rubbish_Speed+5+5, Rubbish_Speed+5+5, 0);
	scaner_set.EdgeIgnore = 0;
	line_pid_param = origin_param;
}

/*长直立景点*/
void view(void)
{
	motor_all.Cspeed = Low_Speed;
	float origin_c = motor_all.Cincrement;

	while (Infrared_ahead == 0) // 撞挡板
		vTaskDelay(5);
	
	Want2Go(10);
	encoder_clear();
	send_play_specified_command(6);
	CarBrake();
	vTaskDelay(100);
	
	mpuZreset(imu.yaw, nodesr.nowNode.angle);  //陀螺仪校正
	encoder_clear();
	Motor_Control(is_Free,-2000,-2000,0);
	Want2Go(7.5);
	CarBrake();
	vTaskDelay(100);
	Turn_Angle_Relative(179);
	while (fabs(angle.AngleT - getAngleZ()) > 2)
		vTaskDelay(2);
	CarBrake();
	vTaskDelay(100);
	motor_pid_clear();
	
	motor_all.Cincrement = 0.05;
	motor_all.Cspeed = nodesr.nowNode.speed;
	pid_mode_switch(is_Line);
		
	nodesr.nowNode.function = NONE;
	motor_all.Cincrement = origin_c;
	nodesr.flag |= 0x04; // 到达路口
}

/*短直立景点*/
void view1()
{
	motor_all.Cspeed = Gyro_Speed;

	/*撞挡板*/
	while (Infrared_ahead == 0)
		vTaskDelay(5);
	Want2Go(10);
	send_play_specified_command(6);
	CarBrake();
	vTaskDelay(200);

	nodesr.nowNode.function = 0;
	nodesr.flag |= 0x04; // 到达路口
}

/*退短直立景点 - 红外检测*/
void back(void)
{
	/*后退一段距离*/
	Motor_Control(is_Free, -2000, -2000, 0);
	if(nodesr.lastNode.nodenum == S5)
		Want2Go(20);
	else if(nodesr.lastNode.nodenum == S4)
		Want2Go(10);
	encoder_clear();
	CarBrake();
	vTaskDelay(100);

	Turn_Angle_Relative(need2turn(getAngleZ(), nodesr.nextNode.angle));
	while (fabs(need2turn(getAngleZ(), nodesr.nextNode.angle)) > 2)
	{
		vTaskDelay(2);
		getline_error();
		if (Scaner.lineNum == 1 && ((Scaner.detail & 0x180) != 0) && (fabs(need2turn(angle.AngleT, getAngleZ())) < fabs(need2turn(angle.AngleT, nodesr.nowNode.angle)) * 0.15f))
			break;
	}
	Motor_Control(is_Line, 28, 28, 0);

	nodesr.nowNode.function = 0;
	nodesr.flag |= 0x04; // 到达路口
}

/*波浪板*/
void Barrier_WavedPlate(float lenght)
{
	struct PID_param origin_param1 = gyroG_pid_param;
	struct PID_param origin_param = line_pid_param;
	
	/*进板前*/
	Motor_Control(is_Line, Low_Speed, Low_Speed, 0);
	while (Scaner.ledNum <= 4 || Scaner.lineNum == 1)
	{
		Cross_getline();
		if((Cross_Scaner.detail & 0x180) == 0x180)
			mpuZreset(imu.yaw,nodesr.nowNode.angle);
		vTaskDelay(2);
	}
	line_pid_param.kp = 35; // 23.5
	line_pid_param.ki = 0;	// 0.004
	line_pid_param.kd = 0;
	/*进板*/
	float num = 0;
	
	/*旧循迹参数*/
	Motor_Control(is_Line, BL_Speed, BL_Speed, 0);
	num = motor_all.Distance;
	scaner_set.EdgeIgnore = 3;
	buzzer_on();
	while (fabsf(motor_all.Distance - num) < lenght)
	{
		vTaskDelay(2);
		CGChange(BL_Speed);
	}

	/*出板*/
	WavePlateLeft_Flag = 0;
	WavePlateRight_Flag = 0;
	scaner_set.EdgeIgnore = 0;
	line_pid_param = origin_param;
	nodesr.nowNode.function = 0;
	buzzer_off();
	gyroG_pid_param = origin_param1;
	nodesr.flag |= 0x04; // 到达路口
}

/*南极*/
void South_Pole(void)
{
	float num = 0;
	uint8_t getZ = 0;
	float origin_turnM = motor_all.GyroT_speedMax;
	struct PID_param origin_param = line_pid_param;
	struct PID_param origin_param1 = gyroG_pid_param;
	motor_all.GyroT_speedMax = 25;
	gyroG_pid_param.kp = 0.5;
	gyroG_pid_param.ki = 0;
	gyroG_pid_param.kd = 0.5;
	
	/*等待识别到坡*/
	encoder_clear();
	num = motor_all.Distance;
	while (fabsf(motor_all.Distance - num) < 40)
	{
		vTaskDelay(2);
		if (Scaner.ledNum >= 4 || Scaner.lineNum >= 2)
			break;
	}
	encoder_clear();

	/*等待开始上坡*/
	Motor_Control(is_Gyro, Low_Speed, Low_Speed, nodesr.nowNode.angle);
	num = motor_all.Distance;
	while (fabsf(motor_all.Distance - num) < 40)
	{
		if (imu.pitch > basic_p + 10)
			break;
		vTaskDelay(2);
	}
	encoder_clear();

	/*开始上坡*/
	Motor_Control(is_Line, Mount_Speed, Mount_Speed, 0);
	Robot_Work(BODY, UP);
	buzzer_on();
	num = motor_all.Distance;
	while (fabsf(motor_all.Distance - num) < 140)
	{
		vTaskDelay(2);
		if((Scaner.detail & 0X180) == 0X180)
		{
			angle.AngleG = getAngleZ();
			getZ = 1;
		}
		if(Scaner.ledNum >= 10 && (fabsf(motor_all.Distance - num) > 50))
			break;
	}
	/*上坡结束*/
	buzzer_off();
	if(getZ == 0)
		angle.AngleG = nodesr.nowNode.angle;
	
	gyroG_pid_param.kp = 4.5;
	gyroG_pid_param.ki = 0;
	gyroG_pid_param.kd = 0;
	Motor_Control(is_Gyro, IMPACT_SPEED, IMPACT_SPEED, angle.AngleG);

	/*撞挡板*/
	while (Infrared_ahead == 0)
		vTaskDelay(5);
	Want2Go(10);
	CarBrake();
	vTaskDelay(200);
	
	mpuZreset(imu.yaw, nodesr.nowNode.angle); // 陀螺仪校正
	Arrived_Stage();

	/*后退一段距离*/
	num = motor_all.Distance;
	Motor_Control(is_Free, -2000, -2000, 0);
	while (fabsf(motor_all.Distance - num) < 5.0f) //7.5
		vTaskDelay(2);
	encoder_clear();
	CarBrake();
	vTaskDelay(100);
	
	send_play_specified_command(12);

	/*180°转*/
	Turn_Angle_Relative(179);
	while (fabs(angle.AngleT - getAngleZ()) > 2)
		vTaskDelay(5);
	motor_pid_clear();

	/*宝藏*/
	if (treasure[2] == 7)
	{
		Robot_Work(LARM, UP);
		Robot_Work(RARM, UP);
		send_play_specified_command(9);
		Turn_Angle360();
		Robot_Work(LARM, DOWN);
		Robot_Work(RARM, DOWN);
	}

	/*开始下坡*/
	buzzer_on();
	Robot_Work(BODY, DOWN);
	Motor_Control(is_Gyro, Rubbish_Speed - 5, Rubbish_Speed - 5, 0);

	do
	{
		vTaskDelay(2);
		getline_error();
	} while (Scaner.ledNum <= 4);
	do
	{
		vTaskDelay(2);
		getline_error();
	} while (Scaner.ledNum >= 4);
	line_pid_param.kp = 35;
	line_pid_param.ki = 0;
	line_pid_param.kd = 1.5;
	Motor_Control(is_Line, Old_M_Speed, Old_M_Speed, 0);
	/*前进一小段*/
	encoder_clear();
	Want2Go(40);

	/*切换循迹，开始下坡*/
	line_pid_param.kp = 12*2;
	line_pid_param.ki = 0;
	line_pid_param.kd = 400;
	Motor_Control(is_Line, UnderMou_Speed, UnderMou_Speed, 0);
	while (imu.pitch < After_down)
		vTaskDelay(2);
	/*下坡结束*/

	buzzer_off();
	line_pid_param = origin_param;
	gyroG_pid_param = origin_param1;
	motor_all.GyroT_speedMax = origin_turnM;
	nodesr.nowNode.function = 0;
	nodesr.flag |= 0x04; // 到达路口
}

/*跷跷板*/
void QQB_1(void)
{
	uint16_t break_time = 0;
	float num;
	struct PID_param origin_param1 = gyroG_pid_param;
	int timeout = 0;
	gyroG_pid_param.kp = 3.5;
	gyroG_pid_param.ki = 0.004;
	gyroG_pid_param.kd = 0;
	motor_all.Cspeed = Low_Speed;
	pid_mode_switch(is_Line);
	if(nodesr.nowNode.nodenum == B9)
		scaner_set.CatchsensorNum = line_weight_default[9]; // 给予左边权值
	else
		scaner_set.CatchsensorNum = line_weight_default[9]; // 给予左边权值
	infrare_open = 1;

	/*板处理*/
	/*循迹走板前1/4弯弧*/
	Want2Go(70);	/*80*/
	motor_all.Cspeed = 15; 
	while ((imu.pitch < basic_p + 6))
	{
		Cross_getline();
		if ((Cross_Scaner.detail & 0x3) == 0x3 ||
			(Cross_Scaner.detail & 0x6) == 0x6 ||
			(Cross_Scaner.detail & 0xC) == 0xC)
			break;
		vTaskDelay(2);
	}
	scaner_set.CatchsensorNum = 0;

	/*陀螺仪转正位置*/
	Motor_Control(is_Gyro, QQB_Speed, QQB_Speed, getAngleZ()+25);
	while (fabs(need2turn(getAngleZ(), angle.AngleG)) > 5)
	{
		break_time++;
		if (break_time > 500)
			break;
		vTaskDelay(2);
	}

	/*板上红外+循迹矫正*/
	gyroG_pid_param.kp = 8;
	gyroG_pid_param.ki = 0.004;
	gyroG_pid_param.kd = 0;
	angle.AngleG = getAngleZ();
	motor_all.Gspeed = QQB_Speed;
	encoder_clear();
	num = motor_all.Distance;
	while (fabsf(motor_all.Distance - num) < 68/*70*/)
	{
		getline_error();
		if ((infrared.head_left == 1 && infrared.head_right == 0) || (Scaner.detail & 0xF800))
			angle.AngleG = getAngleZ() - 3;
		else if ((infrared.head_left == 0 && infrared.head_right == 1) || (Scaner.detail & 0x3F))
			angle.AngleG = getAngleZ() + 3;
		vTaskDelay(2);
	}

	/*板中停车等待板砸下*/
	while(imu.pitch > Down_pitch+6)
	{	
		CarBrake();
		vTaskDelay(2);
		timeout ++;
		if(timeout >= 750)
			break;
	}
	timeout = 0;
	vTaskDelay(500);
	infrare_open = 0;
	scaner_set.EdgeIgnore = 0;
	
	/*开环转一段，确保能看到线*/
	float angle1 = getAngleZ();
	Motor_Control(is_Free, -1500, 1500, 0);

	float needangle = 90;
	if (nodesr.nowNode.nodenum == B8)
	{
		vTaskDelay(250);
		needangle = 75;
	}
	else
	{
		vTaskDelay(500);
		needangle = 90;
	}
	while ((fabsf(need2turn(getAngleZ(), angle1))) < needangle)
	{
		vTaskDelay(2);
		Cross_getline();
		if (Cross_Scaner.lineNum == 1 && Cross_Scaner.ledNum >= 2 && (Cross_Scaner.detail & 0xfff0) && (fabs(need2turn(angle.AngleG, getAngleZ())) < fabs(need2turn(angle.AngleG, needangle) * 0.25f)))
			break;
	}
	
	/*两板子给不同速度*/
	scaner_set.CatchsensorNum = 0;
	motor_all.Cspeed = Low_Speed;
	
	pid_mode_switch(is_Line);
	gyroG_pid_param = origin_param1;
	nodesr.nowNode.function = 0;
	nodesr.flag |= 0X04;
}

/*看灯*/
void door()
{
	static uint8_t flag = 0;
	static uint8_t wait_cnt = 0;
	buzzer_on();

	while (1)
	{
		/*走到灯前*/
		if (Scaner.ledNum >= 8)
		{
			buzzer_off();
			Motor_Control(is_No, 0, 0, 0);
			vTaskDelay(200); 		// 识别红绿

			/*判断开哪边的MV*/
			Color_Right = Color_Left = 0;
			if (flag <= 10) // 去
				Open_MV_R();
			else
				Open_MV_L();

			/*等看完灯*/
			uint16_t outtime = 0;

			// if (nodesr.nowNode.nodenum == N12)
			// 	Color_Right = Red;
			// if (nodesr.nowNode.nodenum == N8)
			// 	Color_Right = Green;

			while (Color_Right == 0 && Color_Left == 0)
			{
				outtime++;
				if (outtime >= 750)
					break;
				vTaskDelay(2);
			}

			/*第一次看灯 - 看D2*/
			if (flag == 0)
			{
				while (1)
				{
					/*红灯 - 回去准备看D3*/
					if (Color_Right == Red)
					{
						color_flag[0] = Color_Right;
						send_play_specified_command(11);
						map.point = 0;
						route[map.point] = N8;

						Turn_Angle_Relative(181);
						while (fabs(angle.AngleT - getAngleZ()) > 2)
						{
							getline_error();
							if (Scaner.lineNum == 1 && ((Scaner.detail & 0x3C0)) != 0 && (fabs(need2turn(angle.AngleT, getAngleZ())) < 27))
							{
								break;
							}
							vTaskDelay(2);
						}

						nodesr.nowNode = Node[getNextConnectNode(N12, N5)];
						nodesr.nowNode.step = 70;	
						nodesr.nowNode.flag = STOPTURN | DRIGHT | DLEFT;
						nodesr.nowNode.speed = SPEED4;
						
						motor_all.Cspeed = nodesr.nowNode.speed;
						pid_mode_switch(is_Line);
						nodesr.flag |= 0x20;
						flag = 1;
						Close_MV_R();
						return;
					}
					/*绿灯 - 继续前进*/
					else if (Color_Right == Green)
					{
						color_flag[0] = Color_Right;
						send_play_specified_command(8);
						map.point = 0;
						nodesr.nowNode = Node[getNextConnectNode(N5, N12)]; // 重新设置nowNode
						nodesr.nowNode.flag = DLEFT | DRIGHT | LEFT_LINE;
						nodesr.nowNode.step = 90 /*100*/;
						nodesr.nowNode.speed = SPEED3;
						nodesr.nowNode.function = NONE;
						
						for(uint8_t i = 0;i<100;i++)
						{
							route[i] = door3route[i];
							if(door3route[i]==0xff)
								break;
						}
						motor_all.Cspeed = nodesr.nowNode.speed;
						pid_mode_switch(is_Line);
						nodesr.flag |= 0x80;
						flag = 0;	//确定路线
						Close_MV_R();
						return;
					}
					/*黄灯 - 继续前进，需要看D5*/
					else if (Color_Right == Yellow)
					{
						send_play_specified_command(10);
						color_flag[0] = Color_Right;
						map.point = 0;
						nodesr.nowNode = Node[getNextConnectNode(N5, N12)]; // 重新设置nowNode
						nodesr.nowNode.flag = DLEFT | DRIGHT;
						nodesr.nowNode.step = 100;
						nodesr.nowNode.speed = SPEED3;
						nodesr.nowNode.function = NONE;
						
						for(uint8_t i = 0;i<100;i++)
						{
							route[i] = door12route[i];
							if(door12route[i]==0xff)
							{
								break;
							}
						}
						motor_all.Cspeed = nodesr.nowNode.speed;
						pid_mode_switch(is_Line);
						nodesr.flag |= 0x80;
						flag = 11;	//需要看D5
						Close_MV_R();
						return;
					}
					/*错误*/
					else
					{
						CarBrake();
					}
					vTaskDelay(2);

					/*超时还没看完灯就再开一次*/
					wait_cnt++;
					if (wait_cnt == 50)
					{
						wait_cnt = 0;
						Open_MV_R();
					}
				}
			}
			/*第二次看灯 - D2红,看D3*/
			if (flag == 1)
			{
				while (1)
				{
					/*红灯 - 回去*/
					if (Color_Right == Red)
					{
						color_flag[1] = Color_Right;
						send_play_specified_command(11);
						map.point = 0;
						Turn_Angle_Relative(181);
						while (fabs(angle.AngleT - getAngleZ()) > 2)
						{
							vTaskDelay(2);
						}
						
						nodesr.nowNode = Node[getNextConnectNode(N8, N5)]; // 重新设置nowNode
						nodesr.nowNode.flag |= CLEFT | CRIGHT;
						nodesr.nowNode.step = 70;
						nodesr.nowNode.speed = SPEED4;
						nodesr.nowNode.function = NONE;
						
						for(uint8_t i = 0;i<100;i++)
						{
							route[i] = door1route[i];
							if(door1route[i]==0xff)
							{
								break;
							}
						}
						//route_reset(1);
						motor_all.Cspeed = nodesr.nowNode.speed;
						pid_mode_switch(is_Line);
						nodesr.flag |= 0x20;
						flag = 2;
						Close_MV_R();
						return;
					}
					/*非红灯 - 继续走*/
					else if (Color_Right == Green | Color_Right == Yellow)
					{
						nodesr.nowNode = Node[getNextConnectNode(N5, N8)]; // 重新设置nowNode
						nodesr.nowNode.flag = DLEFT | LEFT_LINE | DRIGHT;
						nodesr.nowNode.step = 60;
						nodesr.nowNode.speed = SPEED3;
						nodesr.nowNode.function = NONE;
						map.point = 0;
						if (Color_Right == Green)
						{
							color_flag[1] = Color_Right;
							send_play_specified_command(8);
							Node[getNextConnectNode(N8, N5)].function = NONE;
							Node[getNextConnectNode(N8, N5)].speed = SPEED4;
							Node[getNextConnectNode(N8, N5)].step = 150;
							
							for(uint8_t i = 0;i<100;i++)
							{
								route[i] = door2route[i];
								if(door2route[i]==0xff)
									break;
							}
							flag = 0;	//确定路线
						}
						else if (Color_Right == Yellow)
						{
							color_flag[1] = Color_Right;
							send_play_specified_command(10);
							nodesr.flag |= 0x80;
							
							for(uint8_t i = 0;i<100;i++)
							{
								route[i] = door5route[i];
								if(door5route[i]==0xff)
									break;
							}
							flag = 11;	//要看D5
						}
						Motor_Control(is_Line, nodesr.nowNode.speed, nodesr.nowNode.speed, 0);
						Close_MV_R();
						nodesr.flag |= 0x80;
						return;
					}
					else
						CarBrake();
					vTaskDelay(2);
					wait_cnt++;
					if (wait_cnt == 50)
					{
						wait_cnt = 0;
						Open_MV_R();
					}
				}
			}
			/*第三次看灯 - D2红,D3红,看D4*/
			if (flag == 2)
			{
				flag = 0;
				while (1)
				{
					/*绿灯 - 继续前进*/
					if (Color_Right == Green)
					{
						send_play_specified_command(8);
						color_flag[2] = Color_Right;
			
						Node[getNextConnectNode(N8, N3)].function = NONE;
						Node[getNextConnectNode(N8, N3)].speed = SPEED4;
						Node[getNextConnectNode(N8, N3)].step = 150;
						
						/*录入新路径*/
						for(uint8_t i = 0;i<100;i++)
						{
							route[i] = door4route[i];
							if(door4route[i]==0xff)
							{
								break;
							}
						}
						//route_reset(4);
						Close_MV_R();
						break;
					}
					/*黄灯 - 继续前进*/
					else if (Color_Right == Yellow)
					{
						send_play_specified_command(10);
						color_flag[2] = Color_Right;
						Node[getNextConnectNode(N10, N3)].function = NONE;		//D5必定绿，不用看
						Node[getNextConnectNode(N10, N3)].speed = SPEED4;
						Node[getNextConnectNode(N10, N3)].step = 200;
						
						for(uint8_t i = 0;i<100;i++)
						{
							route[i] = door9route[i];
							if(door9route[i]==0xff)
							{
								break;
							}
						}
						//route_reset(9);
						Close_MV_R();
						break;
					}
					else
					{
						CarBrake();
					}
					vTaskDelay(2);
					wait_cnt++;
					if (wait_cnt == 50)
					{
						wait_cnt = 0;
						Open_MV_R();
					}
				}
				map.point = 0;
				nodesr.nowNode = Node[getNextConnectNode(N3, N8)]; // 重新设置nowNode
				nodesr.nowNode.flag = DLEFT | DRIGHT;
				nodesr.nowNode.step = 70;
				nodesr.nowNode.speed = SPEED3;
				nodesr.nowNode.function = NONE;
				nodesr.flag |= 0x80;
				
				motor_all.Cspeed = nodesr.nowNode.speed;
				pid_mode_switch(is_Line);
				nodesr.nowNode.function = 1;
				return;
			}
			/*看D5 - 两种情况：1.D2黄 2.D2红 D3黄*/
			if (flag == 11)
			{
				while (1)
				{
					/*绿灯 - 继续前进*/
					if (Color_Left == Green)
					{
						send_play_specified_command(8);
						color_flag[3] = Color_Left;
						map.point = 0;
						nodesr.nowNode = Node[getNextConnectNode(N10, N3)]; // 重新设置nowNode
						nodesr.nowNode.flag = DRIGHT | RIGHT_LINE;
						nodesr.nowNode.step = 65;
						nodesr.nowNode.speed = SPEED4;
						nodesr.nowNode.function = NONE;
						nodesr.flag |= 0x80;
						
						for(uint8_t i = 0;i<100;i++)
						{
							route[i] = door6route[i];
							if(door6route[i]==0xff)
								break;
						}
						break;
					}
					/*红灯 - 返回看情况*/
					else if (Color_Left == Red)
					{
						send_play_specified_command(11);
						color_flag[3] = Color_Left;
						
						/*D2黄，D5红，去看D4*/
						if (color_flag[0] == Yellow)
						{
							map.point -= 2;
							route[map.point] = N8;
							route[map.point + 1] = N3;

							Turn_Angle_Relative(181);
							while (fabs(angle.AngleT - getAngleZ()) > 2)
							{
								vTaskDelay(2);
							}

							nodesr.nowNode = Node[getNextConnectNode(N3, N10)];
							nodesr.nowNode.step = 70;
							nodesr.nowNode.flag = DRIGHT | DLEFT;
							nodesr.nowNode.speed = SPEED3;
							flag = 12;
						}
						/*D2红，D3黄，D5红，剩下的必定都是绿*/
						else if ((color_flag[0] == Red) && (color_flag[1] == Yellow))
						{
							Turn_Angle_Relative(181);
							while (fabs(angle.AngleT - getAngleZ()) > 2)
							{
								vTaskDelay(2);
							}

							map.point = 0;
							nodesr.nowNode = Node[getNextConnectNode(N3, N10)]; // 重新设置nowNode
							nodesr.nowNode.flag = DLEFT | DRIGHT;
							nodesr.nowNode.step = 70;
							nodesr.nowNode.speed = SPEED3;
							nodesr.nowNode.function = NONE;
							
							Node[getNextConnectNode(N8, N3)].function = NONE;
							Node[getNextConnectNode(N8, N3)].speed = SPEED4;
							Node[getNextConnectNode(N8, N3)].step = 150;
							
							for(uint8_t i = 0;i<100;i++)
							{
								route[i] = door7route[i];
								if(door7route[i]==0xff)
								{
									break;
								}
							}
							//route_reset(7);
							flag = 0;
						}
						nodesr.flag |= 0x20;
						break;
					}
					else
					{
						CarBrake();
					}
					vTaskDelay(2);
					wait_cnt++;
					if (wait_cnt == 50)
					{
						wait_cnt = 0;
						Open_MV_L();
					}
				}
				Close_MV_L();
				pid_mode_switch(is_Line);
				motor_all.Cspeed = nodesr.nowNode.speed;
				nodesr.nowNode.function = 1;
				return;
			}
			/*看D4 - D2黄，D5红，需要看D4确认*/
			if (flag == 12)
			{
				while (1)
				{
					/*绿灯 - 继续前进*/
					if (Color_Left == Green)
					{
						send_play_specified_command(8);
						color_flag[2] = Color_Left;
						map.point = 0;
						nodesr.nowNode = Node[getNextConnectNode(N8, N3)]; // 重新设置nowNode
						nodesr.nowNode.flag = LEFT_LINE | MUL2MUL | STOPTURN;
						nodesr.nowNode.step = 10;
						nodesr.nowNode.speed = SPEED3;
						nodesr.nowNode.function = 1;
						nodesr.flag |= 0x80;
						
						for(uint8_t i = 0;i<100;i++)
						{
							route[i] = door8route[i];
							if(door8route[i]==0xff)
							{
								break;
							}
						}
						//route_reset(8);
						motor_pid_clear();
						break;
					}
					/*红灯 - 剩下的都是绿灯*/
					else if (Color_Left == Red)
					{
						send_play_specified_command(11);
						color_flag[2] = Color_Left;
						Turn_Angle_Relative(181); //	转到当前结点方向
						while (fabs(angle.AngleT - getAngleZ()) > 2)
						{
							vTaskDelay(2);
						}

						Node[getNextConnectNode(N8, N5)].function = NONE;
						Node[getNextConnectNode(N8, N5)].speed = SPEED4;
						Node[getNextConnectNode(N8, N5)].step = 150;
						
						map.point = 0;
						nodesr.nowNode = Node[getNextConnectNode(N3, N8)];
						nodesr.nowNode.flag = STOPTURN | CLEFT | CRIGHT | DRIGHT | DLEFT;
						nodesr.nowNode.speed = SPEED3;
						motor_all.Cspeed = nodesr.nowNode.speed;

						for(uint8_t i = 0;i<100;i++)
						{
							route[i] = door11route[i];
							if(door11route[i]==0xff)
							{
								break;
							}
						}
						//route_reset(11);
						nodesr.flag |= 0x20;
						Close_MV_L();
						break;
					}
					else
					{
						CarBrake();
					}
					vTaskDelay(2);
					wait_cnt++;
					if (wait_cnt == 50)
					{
						wait_cnt = 0;
						Open_MV_L();
					}
				}
				flag = 0;
				Close_MV_L();
				pid_mode_switch(is_Line);
				motor_all.Cspeed = nodesr.nowNode.speed;
				nodesr.nowNode.function = NONE;
				return;
			}
		}
	}
}

/*珠峰下通道处理*/
void undermou(void)
{
	encoder_clear();
	motor_all.Cspeed = UnderMou_Speed;
	infrare_open = 1;
	while(infrared.head_left == 1 && infrared.head_right == 1)
		vTaskDelay(2);

	buzzer_on();
	encoder_clear();

	if (nodesr.nowNode.nodenum == C8)
		motor_all.Cspeed = Low_Speed;
	else if (nodesr.nowNode.nodenum == N14)
	{
		motor_all.Cspeed = Low_Speed;
		Want2Go(100);
		motor_all.Cspeed = nodesr.nowNode.speed;
	}

	Cross_getline();
	while(!deal_arrive())
	{
		Cross_getline();
		vTaskDelay(2);
	}
	nodesr.nowNode.function = 0;
	nodesr.flag |= 0x04; // 到达路口
}

/*忽略节点 - 直接判定到达路口*/
void ignore_node(void)
{
	nodesr.nowNode.function = 0;
	nodesr.flag |= 0x04; // 到达路口
}

/*第二轮路线规划*/
void get_newroute(void)
{
	mapInit1();
	map.point = 0;
	for (int i = 0; i < 126; i++)
	{
		if (Node[i].function == DOOR)
		{
			Node[i].function = NONE;
			Node[i].step *= 2;
			Node[i].speed = SPEED3;
		}
	}

	if(color_flag[0]==Green)//第一个门开
	{
		Node[getNextConnectNode(P3, N3)].flag |= STOPTURN;
		u8 temp[100]={B1,N1,P1,N1,B2,N4,N5,N6,S2,N6,P4,N6,N5,N12,N13,P6,N13,N12,N16,N18,B5,N19,C6,B7,N22,C9,P8,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N11,N12,N5,N4,N3,P3,N3,S1,N3,N4,B3,N2,P2,0XFF};
		for(int i=0;i<100;i++)
		{
			route[i]=temp[i];
			if(temp[i]==0xff)
				break;
		}
	}
	else if(color_flag[0]==Yellow && color_flag[3]==Green)
	{
		Node[getNextConnectNode(S1, N3)].flag |= STOPTURN;
		u8 temp[100]={B1,N1,P1,N1,B2,N4,N5,N6,S2,N6,P4,N6,N5,N12,N13,P6,N13,N12,N16,N18,B5,N19,C6,B7,N22,C9,P8,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N3,S1,N3,P3,N3,N4,B3,N2,P2,0XFF};
		for(int i=0;i<100;i++)
		{
			route[i]=temp[i];
			if(temp[i]==0xff)
				break;
		}
	}
	else if(color_flag[0]==Yellow && color_flag[3]==Red && color_flag[2]==Green)
	{
		Node[getNextConnectNode(S1, N3)].flag |= STOPTURN;
		u8 temp[100]={B1,N1,P1,N1,B2,N4,N5,N6,S2,N6,P4,N6,N5,N12,N13,P6,N13,N12,N16,N18,B5,N19,C6,B7,N22,C9,P8,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N3,S1,N3,P3,N3,N4,B3,N2,P2,0XFF};
		for(int i=0;i<100;i++)
		{
			route[i]=temp[i];
			if(temp[i]==0xff)
				break;
		}
	} 
	else if(color_flag[0]==Yellow && color_flag[3]==Red && color_flag[2]==Red)
	{
		Node[getNextConnectNode(P3, N3)].flag |= STOPTURN;
		u8 temp[100]={B1,N1,P1,N1,B2,N4,N5,N6,S2,N6,P4,N6,N5,N12,N13,P6,N13,N12,N16,N18,B5,N19,C6,B7,N22,C9,P8,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N5,N4,N3,P3,N3,S1,N3,N4,B3,N2,P2,0XFF};
		for(int i=0;i<100;i++)
		{
			route[i]=temp[i];
			if(temp[i]==0xff)
				break;
		}
	}
	else if(color_flag[0]==Red && color_flag[1]==Green)
	{
		Node[getNextConnectNode(P3, N3)].flag |= STOPTURN;
		u8 temp[100]={B1,N1,P1,N1,B2,N4,N5,N6,S2,N6,P4,N6,N5,N8,N12,N13,P6,N13,N12,N16,N18,B5,N19,C6,B7,N22,C9,P8,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N5,N4,N3,P3,N3,S1,N3,N4,B3,N2,P2,0XFF};
		for(int i=0;i<100;i++)
		{
			route[i]=temp[i];
			if(temp[i]==0xff)
				break;
		}
	}
	else if(color_flag[0]==Red && color_flag[1]==Yellow && color_flag[3]==Green)
	{
		Node[getNextConnectNode(S1, N3)].flag |= STOPTURN;
		u8 temp[100]={B1,N1,P1,N1,B2,N4,N5,N6,S2,N6,P4,N6,N5,N8,N12,N13,P6,N13,N12,N16,N18,B5,N19,C6,B7,N22,C9,P8,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N3,S1,N3,P3,N3,N4,B3,N2,P2,0XFF};
		for(int i=0;i<100;i++)
		{
			route[i]=temp[i];
			if(temp[i]==0xff)
				break;
		}
	}
	else if(color_flag[0]==Red && color_flag[1]==Yellow && color_flag[3]==3)
	{
		Node[getNextConnectNode(S1, N3)].flag |= STOPTURN;
		u8 temp[100]={B1,N1,P1,N1,B2,N4,N5,N6,S2,N6,P4,N6,N5,N8,N12,N13,P6,N13,N12,N16,N18,B5,N19,C6,B7,N22,C9,P8,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N3,S1,N3,P3,N3,N4,B3,N2,P2,0XFF};
		for(int i=0;i<100;i++)
		{
			route[i]=temp[i];
			if(temp[i]==0xff)
				break;
		}
	}
	else if(color_flag[0]==Red && color_flag[1]==Red && color_flag[2]==Green)//从最外面出去吧
	{
		Node[getNextConnectNode(P3, N3)].flag |= STOPTURN;
		u8 temp[100]={B1,N1,P1,N1,B2,N4,N5,N6,S2,N6,P4,N6,N5,N4,N3,P3,N3,S1,N3,N8,N12,N13,P6,N13,N12,N16,N18,B5,N19,C6,B7,N22,C9,P8,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N3,N4,B3,N2,P2,0XFF};

		for(int i=0;i<100;i++)
		{
			route[i]=temp[i];
			if(temp[i]==0xff)
				break;
		}
	}
	else if(color_flag[0]==Red && color_flag[1]==Red && color_flag[2]==Yellow)//从最外面出去吧
	{
		Node[getNextConnectNode(P3, N3)].flag |= STOPTURN;
		u8 temp[100]={B1,N1,P1,N1,B2,N4,N5,N6,S2,N6,P4,N6,N5,N4,N3,P3,N3,S1,N3,N8,N12,N13,P6,N13,N12,N16,N18,B5,N19,C6,B7,N22,C9,P8,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N3,N4,B3,N2,P2,0XFF};
		for(int i=0;i<100;i++)
		{
			route[i]=temp[i];
			if(temp[i]==0xff)
				break;
		}
	}
	else
		CarBrake_Stop();
}

/*K210读数字*/
uint8_t WaitForK210(void)
{
	static uint8_t No2Tra = 0;
	static uint8_t No3Tra = 0;

	if (((nodesr.nowNode.nodenum == P3 || nodesr.nowNode.nodenum == P4) && No2Tra == 1) ||
		((nodesr.nowNode.nodenum == P5 || nodesr.nowNode.nodenum == P6) && No3Tra == 1))
		return 0;

	/*等待看完*/
	uint16_t break_times = 0;
	uint8_t ReturnFlag = 0;
	while (K210_Rece == 0)
	{
		vTaskDelay(3);
		break_times++;
		if (break_times >= 500)
		{
			Motor_Control(is_No, BACK_SPEED, BACK_SPEED, 0);
			Want2Go(1);
			CarBrake();
			encoder_clear();
			open_K210();
			break_times = 0;
			ReturnFlag++;
			if(ReturnFlag > 3)
			{
				// send_play_specified_command(9);
				return 0;
			}
				
		}
	}
	K210_Rece = 0;

	/*记录宝藏*/
	if (nodesr.nowNode.nodenum == P1)
	{
		treasure[0] = Clue_Num;
		Clue_Num = 0;
	}
	else if (nodesr.nowNode.nodenum == P3 || nodesr.nowNode.nodenum == P4)
	{
		No2Tra = 1;
		treasure[1] = Clue_Num;
		Clue_Num = 0;
	}
	else if (nodesr.nowNode.nodenum == P5 || nodesr.nowNode.nodenum == P6)
	{
		No3Tra = 1;
		treasure[2] = Clue_Num;
		Clue_Num = 0;
	}

	/*关闭K210*/
	close_K210();
	buzzer_on();
	vTaskDelay(100);
	buzzer_off();
	return 0;
}

extern uint8_t isAllRoute;
/*启动流程*/
void zhunbei(void)
{
	/*停车*/
	Motor_Control(is_No, 0, 0, 0);

	/*机器人动作*/
	Robot_Work(BODY, UP); 	//人站起来
	vTaskDelay(1000);

	/*蜂鸣器提示初始化完成 - 调试用*/
	buzzer_on();
	vTaskDelay(100);
	buzzer_off();
	infrare_open = 1;

//	/*等待挡板*/
//	while (Infrared_ahead == 0)
//		vTaskDelay(5);

//	/*等待移除挡板*/
//	while(Infrared_ahead == 1)
//		vTaskDelay(5);

	/*播报语音*/
	send_play_specified_command(7);

	/*机器人动作*/
	Robot_Work(LARM, UP);		// 左手举起
	Robot_Work(RARM, UP);		//右手举起
	vTaskDelay(100);
	Robot_Work(LARM, DOWN);		//左手放下
	Robot_Work(RARM, DOWN);		//右手放下
	Robot_Work(BODY, DOWN);		//人躺下

	motor_all.Gincrement = 0.5;
	motor_all.Cincrement = 0.5;

	if(isAllRoute || map.routetime!=0)
	{
		Motor_Control(is_Gyro, Rubbish_Speed, Rubbish_Speed, getAngleZ()); // 设置自平衡及其速度

		LiuShuiRate = LiuShuiRate_BG;
		DownLiuShui = 1;
		/*等待开始下桥*/
		while (imu.pitch > Down_pitch)
			vTaskDelay(2);
		/*正在下桥*/
		while (imu.pitch < After_down)
			vTaskDelay(2);
		/*下桥完毕*/
		DownLiuShui = 0;
		LiuShuiRate = LiuShuiRate_Default;
	}
}

/*保护机制*/
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
	while (fabsf(motor_all.Distance - num) < 5) // 10
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

/*与ConnectFirstBack共用*/
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

/*特殊结点*/
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

/*楼梯陀螺仪循迹自切换 - 使用前需在可靠位置mpuZreset*/
void CGChange(float Speed)
{
	Cross_getline();
	if(Cross_Scaner.ledNum >= 4 || Cross_Scaner.ledNum == 0 || Cross_Scaner.lineNum > 1)
	{
		Motor_Control(is_Gyro, Speed, Speed, nodesr.nowNode.angle);
		//buzzer_off();
	}
	else
	{
		Motor_Control(is_Line, Speed, Speed, 0);
		//buzzer_on();
	}
}

/*快速模式切换
	is_Turn 模式 仅仅aim有效 目标角度
	is_Line 模式 四个参数有效 LSPEED = RSPEED aim为循迹偏置
	is_Gyro 模式 四个参数有效 LSPEED = RSPEED aim为陀螺仪旋转目标值
	is_Free和is_NO 模式 前三个参数有效
*/
void Motor_Control(uint8_t target_mode,float LSPEED,float RSPEED,float aim)
{
	switch(target_mode)
	{
		/*原地转*/
		case is_Turn:
		{
			Turn_Angle_Relative(aim);
			break;
		}
		
		/*循迹模式*/
		case is_Line:
		{
			scaner_set.CatchsensorNum = aim;
			pid_mode_switch(is_Line);
			if(fabs(LSPEED-RSPEED)>1) //防止有人不听话
			{
				motor_all.Cspeed = 0;
				// Kabuto_Audio(7);   						//准备完毕
			}
			else
			{
				motor_all.Cspeed = LSPEED;
			}
			break;
		}
		
		/*陀螺仪自平衡模式*/
		case is_Gyro:
		{
			// mpuZreset(imu.yaw,nodesr.nowNode.angle);
			angle.AngleG = aim;	
			pid_mode_switch(is_Gyro); 					//使用陀螺仪
			if(fabs(LSPEED-RSPEED)>1) 					//防止有人不听话
				motor_all.Gspeed = 0;
			else
				motor_all.Gspeed = LSPEED;
			break;
		}
		
		/*空模式*/
		case is_Free:
		{
			pid_mode_switch(is_Free);

			motor_set_pwm(1,(int32_t)LSPEED);
			motor_set_pwm(2,(int32_t)LSPEED);
			motor_set_pwm(3,(int32_t)RSPEED);
			motor_set_pwm(4,(int32_t)RSPEED);

			break;
		}
		
		/*归零 - 用于停车等*/
		case is_No:
		{
			pid_mode_switch(is_No);
			motor_all.Lspeed = LSPEED;
			motor_all.Rspeed = RSPEED;
			break;
		}
	}
}

/*自定义距离前进*/
void Want2Go(float Dis)
{
	float num = motor_all.Distance;
	while (fabsf(motor_all.Distance - num) < Dis)
		vTaskDelay(2);
}
