#include "map.h"
#include "barrier.h"
#include "sys.h"
#include "usart.h"
#include "QR.h"
#include "delay.h"
#include "scaner.h"
#include "imu.h"
#include "turn.h"
#include "speed_ctrl.h"
#include "motor_task.h"
#include "bsp_linefollower.h"
#include "math.h"
#include "bsp_buzzer.h"
#include "motor_task.h"
#include "encoder.h"
#include "uart.h"
#include "openmv.h"
#include "motor.h"
#include "scaner.h"
struct Map_State map = {0,0};
uint8_t Turn_Flag = 0;
uint8_t Change_Route = 0;
uint8_t mul2sing = 0, sing2mul = 0;

/*D2、D3红，衔接去看D4*/
u8 door1route[100] = {N4, N3, P3, N3, N8, 0XFF};
/*D2红 D3绿*/
u8 door2route[100] = {N12, N13, P6, N13, N12, N16 /*, S5, N16*/, N18, B5, N19, C6, B7, N22, C9, P8, C9, N22, B6, N20, P7, N20, C4, C8, C7, N14, C3, N9, B9, N7, P5, N7, B8, N9, N10 /*, N15, S4, N15, N10*/, N8, N5, N4, N3, P3, N3, N4, B3, N2, P2, 0XFF};
/*D2绿*/
u8 door3route[100] = {N13, P6, N13, N12, N16 /*, S5, N16*/, N18, B5, N19, C6, B7, N22, C9, P8, C9, N22, B6, N20, P7, N20, C4, C8, C7, N14, C3, N9, B9, N7, P5, N7, B8, N9, N10 /*, N15, S4, N15, N10*/, N11, N12, N5, N4, N3, P3, N3, N4, B3, N2, P2, 0XFF};
/*D2红 D3红 D4绿*/
u8 door4route[100] = {N12, N13, P6, N13, N12, N16 /*, S5, N16*/, N18, B5, N19, C6, B7, N22, C9, P8, C9, N22, B6, N20, P7, N20, C4, C8, C7, N14, C3, N9, B9, N7, P5, N7, B8, N9, N10 /*, N15, S4, N15, N10*/, N8, N3, N4, B3, N2, P2, 0XFF};
/*D2红 D3黄，衔接去看D5*/
u8 door5route[100] = {N12, N13, P6, N13, N12, N16 /*, S5, N16*/, N18, B5, N19, C6, B7, N22, C9, P8, C9, N22, B6, N20, P7, N20, C4, C8, C7, N14, C3, N9, B9, N7, P5, N7, B8, N9, N10 /*, N15, S4, N15, N10*/, N3, 0XFF};
/*D5绿*/
u8 door6route[100] = {P3, N3, N4, B3, N2, P2, 0XFF};
/*D2红 D3黄 D5红*/
u8 door7route[100] = {N8, N3, P3, N3, N4, B3, N2, P2, 0xFF};
/*D2黄 D5红 D4绿*/
u8 door8route[100] = {P3, N3, N4, B3, N2, P2, 0XFF};
/*D2红 D3红 D4黄 D5必绿*/
u8 door9route[100] = {N12, N13, P6, N13, N12, N16 /*, S5, N16*/, N18, B5, N19, C6, B7, N22, C9, P8, C9, N22, B6, N20, P7, N20, C4, C8, C7, N14, C3, N9, B9, N7, P5, N7, B8, N9, N10 /*, N15, S4, N15, N10*/, N3, N4, B3, N2, P2, 0XFF}; // 没去P3
/*D2红 D3黄，衔接去看D5*/
u8 door10route[100] = {N12, N13, P6, N13, N12, N16, N18, B5, N19, C6, B7, N22, C9, P8, C9, N22, B6, N20, P7, N20, C4, C8, C7, N14, C3, N9, B9, N7, P5, N7, B8, N9, N10 /*, N15, S4, N15, N10*/, N3, 0XFF}; // N3截至，没回家，没P3
/*D2黄 D5红 D4红*/
u8 door11route[100] = {N5, N4, N3, P3, N3 ,N4, B3, N2, P2, 0XFF};
/*D2黄*/
u8 door12route[100] = {N13, P6, N13, N12, N16 /*, S5, N16*/, N18, B5, N19, C6, B7, N22, C9, P8, C9, N22, B6, N20, P7, N20, C4, C8, C7, N14, C3, N9, B9, N7, P5, N7, B8, N9, N10 /*, N15, S4, N15, N10*/, N3, 0XFF};

/*
	运作中间变量
	flag 0位：1编码器清零请求，0清零完毕
	flag 1位：启动路口判断
	flag 2位：是否到达路口
	flag 3位：arrive里temp清零
	flag 4位：Z轴置零
	flag 5位：路线处理复位 打到门
	flag 6位：没有门
	flag 7位：红灯
*/
NODESR nodesr;

/*全程*/
uint8_t isAllRoute = 1;
u8 route[100] = {B1, N1, P1, N1, B2, N4, N5, N6, P4, N6, N5, N12, 0XFF};
/*测试*/
// uint8_t isAllRoute = 0;
// u8 route[100] = {B6, N20, P7, N20, 0XFF};

/*地图初始化*/
void mapInit()
{
	nodesr.flag = 0;

	// /***************测试***************/
	// nodesr.lastNode.nodenum = C9;
	// nodesr.nextNode.nodenum = B6;
	// nodesr.nowNode = Node[getNextConnectNode(C9, N22)];

	/***************出家***************/
	nodesr.nowNode.nodenum = N2;
	nodesr.nowNode.angle = 0;
	nodesr.nowNode.function = 1;
	nodesr.nowNode.speed = SPEED1;
	nodesr.nowNode.step= 10;
	nodesr.nowNode.flag = CLEFT|RIGHT_LINE;
}

/*第二次出发初始化*/
void mapInit1(void)
{
	nodesr.flag = 0;
	nodesr.nowNode.nodenum = N2;				//起始点   		//N2
	nodesr.nowNode.angle = 0;					//起始角度   	//0
	nodesr.nowNode.function = 1;				//起始函数   	//1
	nodesr.nowNode.speed = SPEED0;				// 300
	nodesr.nowNode.step= 10;					//60               
	nodesr.nowNode.flag = CLEFT|RIGHT_LINE;		//CLEFT|RIGHT_LINE
}
	
/*得到本节点到相邻结点的地址 - 返回第一参数到第二参数处理方法即第二参数*/
u8 getNextConnectNode(u8 nownode,u8 nextnode) 
{
	unsigned char rest = ConnectionNum[nownode];	//这个结点相邻的结点数
	unsigned char addr = Address[nownode];			//得到结点的addr
	int i = 0;
	for (i = 0; i < rest; i++) 
	{
		if(Node[addr].nodenum == nextnode)			//返回结点地址	
			return addr;
		addr++;
	}
	return 0;
}

/*判断节点*/
u8 deal_arrive()
{				
	register uint8_t lnum = 0, i = 0;
	register uint16_t seed = 0;

	if ((nodesr.nowNode.flag & DLEFT) == DLEFT)  //左半边
	{
		//左边6个灯任意5个亮即可
		if (Cross_Scaner.ledNum>=5)
		{
			seed = 0X8000;
			for (i = 0; i<6; i++)
			{
				if (Cross_Scaner.detail & seed)
					++lnum;
				if (lnum >= 5)
				{
					return 1;
				}
				seed >>= 1;
			}
			lnum = 0;
		}
	}
	if ((nodesr.nowNode.flag & DRIGHT) == DRIGHT)//右半边
	{    	
		if (Cross_Scaner.ledNum >= 5)
		{
			seed = 0X0001;
			for (i = 0; i<6; i++)
			{
				if (Cross_Scaner.detail & seed)
					++lnum;
				if (lnum >= 5)
				{
					return 1;
				}
				seed <<= 1;
			}
			lnum = 0;
		}
	}
	if ((nodesr.nowNode.flag & CLEFT) == CLEFT)//左分岔路
	{
		//左边数起第二、第三个灯任意一个亮即可
		 if( (Cross_Scaner.ledNum>=4&&Cross_Scaner.ledNum<=7) && ((Cross_Scaner.detail&0x4000)|(Cross_Scaner.detail&0x2000)) )//
		 {
			return 1;
		}
	}
	if ((nodesr.nowNode.flag & MCLEFT) == MCLEFT)//左分岔路
	{
		//左边数起第一个灯亮即可
		 if( (Cross_Scaner.ledNum>=4&&Cross_Scaner.ledNum<=7) && (Cross_Scaner.detail&0x8000) )
		{
			return 1;
		}
	}
	if ((nodesr.nowNode.flag & MCRIGHT) == MCRIGHT)//左分岔路
	{
		//右边数起第一个灯亮即可
		 if( (Cross_Scaner.ledNum>=4&&Cross_Scaner.ledNum<=7) && (Cross_Scaner.detail&0x0001) )
		{
			return 1;
		}
	}
	if ((nodesr.nowNode.flag & CRIGHT) == CRIGHT)//右分岔路
	{
		 if( (Cross_Scaner.ledNum>=4&&Cross_Scaner.ledNum<=7) && (Cross_Scaner.detail&0xc) )//右起2和3灯亮
		{
			return 1;
		}
	}
	if ((nodesr.nowNode.flag & MORELED) == MORELED)
	{
		 if( (Cross_Scaner.ledNum>=5) )//5个灯以上亮
		{
			return 1;
		}
	}
	if ((nodesr.nowNode.flag & AWHITE) == AWHITE)//全白
	{
		 if((Cross_Scaner.ledNum>=10&&(Cross_Scaner.detail&0x1FF8)==0x1FF8))
		{
			return 1;
		}
	}
	if ((nodesr.nowNode.flag & MUL2SING) == MUL2SING)//三分岔路
	{
		if (Cross_Scaner.lineNum > 1 && Cross_Scaner.ledNum >= 4)
			++mul2sing;
		if (mul2sing > 4 && Cross_Scaner.lineNum == 1) //线数目由多变成一条
		{
			mul2sing = sing2mul = 0;
			return 1;
		}
	}
	if ((nodesr.nowNode.flag & MUL2MUL) == MUL2MUL)  //线数目由多条变多条
	{
		if (Cross_Scaner.lineNum > 1 && Cross_Scaner.ledNum >= 4)
			++mul2sing;
		if (mul2sing > 4 && (Cross_Scaner.lineNum == 1 || Cross_Scaner.ledNum <=3 ))
			++sing2mul;
		if (sing2mul > 4 && Cross_Scaner.lineNum > 1 && Cross_Scaner.ledNum >= 4)
		{
			mul2sing = sing2mul = 0;
			return 1;
		}
	}
	
	return 0;
}

/*节点间处理*/
void Cross(void)
{
	static uint8_t half_times = 0;		//是否完成半程
	static uint8_t _flag = 1;			//结点动作标志位
	static uint8_t ErrorTimes[2] = {0};	//游龙判定计数位

	/*位于起点处理*/
	if(map.point == 0)
	{
		if (isAllRoute == 0)
			motor_all.Cspeed = nodesr.nowNode.speed;
		nodesr.nextNode = Node[getNextConnectNode(nodesr.nowNode.nodenum, route[map.point++])]; // 获取下一结点
	}

	/*节点前段循迹处理*/
	if(_flag == 1)
	{
		if (nodesr.lastNode.nodenum == N4 && nodesr.nowNode.nodenum == N5 && nodesr.nextNode.nodenum == N6 ||
			nodesr.lastNode.nodenum == N5 && nodesr.nowNode.nodenum == N6 && nodesr.nextNode.nodenum == P4 ||
			nodesr.lastNode.nodenum == N5 && nodesr.nowNode.nodenum == N4 && nodesr.nextNode.nodenum == N3 ||
			nodesr.lastNode.nodenum == P3 && nodesr.nowNode.nodenum == N3 && nodesr.nextNode.nodenum == N4)
		{
			nodesr.nowNode.speed = SPEED4;
			motor_all.Cspeed = nodesr.nowNode.speed;
		}

		/*根据接下来的速度给不同的PID参数*/
		if (nodesr.nowNode.speed == SPEED4 && ErrorTimes[0] < 5)
		{
			line_pid_param.kp = 3;
			line_pid_param.ki = 0;
			line_pid_param.kd = 125;
		}
		else if (nodesr.nowNode.speed == SPEED3 && ErrorTimes[0] < 5)
		{
			line_pid_param.kp = 5;
			line_pid_param.ki = 0;
			line_pid_param.kd = 200;
		}
		else
		{
			line_pid_param.kp = 12;
			line_pid_param.ki = 0;
			line_pid_param.kd = 400;
		}
		pid_mode_switch(is_Line);

		/*点间循迹模式切换*/
		if(fabsf(motor_all.Distance) >= 0.5f*nodesr.nowNode.step && half_times == 0)
		{
			if((nodesr.nowNode.flag & Temp_L) == Temp_L)
			{
				LEFT_RIGHT_LINE = 1;
			}
			else if((nodesr.nowNode.flag & Temp_R) == Temp_R)
			{
				LEFT_RIGHT_LINE = 2;
			}
			else if((nodesr.nowNode.flag & Temp_LiuShui) == Temp_LiuShui)
			{
				LEFT_RIGHT_LINE = 3;
			}
			half_times = 1;
		}
		/*节点间后段处理 - 走完70%路程*/
	    else if(fabsf(motor_all.Distance) >= 0.7f*nodesr.nowNode.step)
		{
			_flag = 0;

			if ((fabs(need2turn(getAngleZ(), nodesr.nextNode.angle)) < 10) || (fabs(need2turn(nodesr.nowNode.angle, nodesr.nextNode.angle)) < 10) || (nodesr.nowNode.flag & NOTURN) == NOTURN)
				motor_all.Cspeed = nodesr.nowNode.speed;	//不用减速转弯，靠循迹转弯
			else
			{
				/*返回低速PID参数*/
				line_pid_param.kp = 12;
				line_pid_param.ki = 0;
				line_pid_param.kd = 400;

				motor_all.Cspeed = Gyro_Speed;
			}

			/*归零游龙保护*/
			ErrorTimes[0] = ErrorTimes[1] = 0;
		}
		/*前段路程处理 - 还没走完70%路程*/
		else
		{
			/*游龙判断与保护*/
			if (nodesr.nowNode.angle != nodesr.lastNode.angle && nodesr.nowNode.nodenum != C4 && nodesr.nowNode.nodenum != P8)
			{
				Cross_getline();
				/*危险区 - 一旦进入危险区，安全位置0*/
				if (Cross_Scaner.detail & 0xFC3F)
				{
					ErrorTimes[0]++;
					ErrorTimes[1] = 0;
				}
				else
				{
					if(Cross_Scaner.detail & 0x180)
						ErrorTimes[1] += 5;
					else
						ErrorTimes[1] += 1;
				}
				/*安全区 - 一直处于安全区，危险位置0*/
				if (ErrorTimes[1] >= 25)
				{
					ErrorTimes[0] = 0;
					motor_all.Cspeed = nodesr.nowNode.speed;
				}
				/*非常危险，进入游龙保护*/
				if (ErrorTimes[0] >= 5)
				{
					motor_all.Cspeed = nodesr.nowNode.speed / 1.5f;
				}
			}
		}
	}
	/*节点后端处理 - 处理函数、识别路口、转弯等*/
	else if(_flag == 0)
	{
		/*调用处理函数*/
		map_function(nodesr.nowNode.function);
		
		/*判断路口 - 还没到路口且没有看灯*/
		if((nodesr.flag&0x04)!=0x04 && (nodesr.flag&0x80)!=0x80 && (nodesr.flag&0x20)!=0x20)
		{
			/*↓ 回家N2路口处执行此范围内的代码后会加速 ↓*/
			Cross_getline();
			while(!deal_arrive())
			{
				vTaskDelay(2);
				Cross_getline();
				if(((nodesr.nowNode.flag&RESTMPUZ) == RESTMPUZ))		//陀螺仪校正
				{
					if((Cross_Scaner.detail & 0X0180) == 0X0180) 		//如果在最中间位置
						mpuZreset(imu.yaw, nodesr.nowNode.angle);     	//获取补偿角Z;
				}
			}
			/*↑ 回家N2路口处执行此范围内的代码后会加速 ↑*/
			if(nodesr.nowNode.nodenum == N2 && nodesr.nextNode.nodenum == P2)
				Motor_Control(is_Line,nodesr.nowNode.speed*0.9f,nodesr.nowNode.speed*0.9f,0);

			send_play_specified_command(13);							//播报识别到路口
			scaner_set.CatchsensorNum = 0;
			nodesr.flag |= 0x04;										//标记到达路口
		}
	}

	/*转弯处理 与 继承一节点*/
	if((nodesr.flag&0x04)==0x04)
	{
		nodesr.flag&=~0x04;												//	清除到达路口标志

		/*不是路线末端 - 0xFF*/
		if(route[map.point-1] != 0xFF)
		{
		  	/*如果下一结点角度与当前结点角度相同，不需要转，不需要减速*/   
			if ((fabs(need2turn(getAngleZ(),nodesr.nextNode.angle))<10) 
				||(fabs(need2turn(nodesr.nowNode.angle,nodesr.nextNode.angle))<10)
				||(nodesr.nowNode.flag&NOTURN)==NOTURN
				||(nodesr.nowNode.nodenum==S1)
				||(nodesr.nowNode.nodenum==S2)
				||(route[map.point-3]==S3)
				||(route[map.point-3]==S4)
				||(route[map.point-3]==S5)
				||(isStage == 1))
			{
				isStage = 0;
				_flag = 1;

				/*长直线*/
				if ((nodesr.nowNode.nodenum == N3 && nodesr.nextNode.nodenum == P3) ||
					(nodesr.nowNode.nodenum == N4 && nodesr.nextNode.nodenum == N3) ||
					(nodesr.nowNode.nodenum == N4 && nodesr.nextNode.nodenum == N5))
				{
					Motor_Control(is_Gyro, nodesr.nextNode.speed, nodesr.nextNode.speed, getAngleZ());
					Want2Go(10);
					Motor_Control(is_Line, nodesr.nextNode.speed, nodesr.nextNode.speed, 0);
				}
				else if ((nodesr.nowNode.nodenum == N13 && nodesr.nextNode.nodenum == P6) ||
						 (nodesr.nowNode.nodenum == N1 && nodesr.nextNode.nodenum == P1))
				{
					scaner_set.EdgeIgnore = 6;
					Want2Go(20);
					scaner_set.EdgeIgnore = 0;
				}
				else if ((nodesr.lastNode.nodenum == P3 && nodesr.nowNode.nodenum == N3 && nodesr.nextNode.nodenum == N4) ||
						 (nodesr.lastNode.nodenum == P4 && nodesr.nowNode.nodenum == N6 && nodesr.nextNode.nodenum == N5))
				{
					scaner_set.EdgeIgnore = 7;
					Want2Go(20);
					scaner_set.EdgeIgnore = 0;
				}
				else if (nodesr.nowNode.nodenum == N13 && nodesr.nextNode.nodenum == N12)
				{
					Motor_Control(is_Gyro, nodesr.nextNode.speed, nodesr.nextNode.speed, getAngleZ());
					Want2Go(5);
					Motor_Control(is_Line, nodesr.nextNode.speed, nodesr.nextNode.speed, 0);
				}
			}
			/*需要转弯*/
			else
			{	 
				if(nodesr.nowNode.flag&L_follow)			//左循迹加大kp的转弯，适用小角度，多线干扰陀螺仪摆尾位置不合适
				{
					float original_line_pid = line_pid_param.kp;
					float max = line_pid_param.outputMax;
					line_pid_param.kp = 70;
					line_pid_param.ki = 0;
					line_pid_param.kd = 5;
					line_pid_param.outputMax = 0.75f*motor_all.Cspeed;
					nodesr.nowNode.flag|=LEFT_LINE;
					angle.AngleG = nodesr.nextNode.angle;
					while(fabs(need2turn(angle.AngleG,getAngleZ()))>4)
					{
						vTaskDelay(2);
						getline_error();
						if(Scaner.lineNum==1&&((Scaner.detail&0x3C0)!=0)&&(fabsf(need2turn(angle.AngleG,getAngleZ()))<fabsf(need2turn(angle.AngleG,nodesr.nowNode.angle))*0.25f))
							break;
					}
					nodesr.nowNode.flag&=(~LEFT_LINE);		//取消左循迹标志位
					line_pid_param.kp = original_line_pid;  //恢复正常
					line_pid_param.outputMax =  max;
				}
				else if(nodesr.nowNode.flag&R_follow)		//右循迹加大kp的转弯，适用小角度，多线干扰陀螺仪摆尾位置不合适
				{
					float original_line_pid = line_pid_param.kp;
					float max = line_pid_param.outputMax;
					float num=motor_all.Distance;
					line_pid_param.kp = 70;
					line_pid_param.ki = 0;
					line_pid_param.kd = 5;
					line_pid_param.outputMax = 0.75f*motor_all.Cspeed;
					nodesr.nowNode.flag|=RIGHT_LINE;
					angle.AngleG = nodesr.nextNode.angle;
					while(fabs(need2turn(angle.AngleG,getAngleZ()))>4)
					{
						vTaskDelay(2);
						getline_error();
						if(Scaner.lineNum==1&&((Scaner.detail&0x3C0)!=0)&&(fabsf(need2turn(angle.AngleG,getAngleZ()))<fabs(need2turn(angle.AngleG,nodesr.nowNode.angle))*0.25f))
						{  
							break;
						}
					}
					nodesr.nowNode.flag&=(~RIGHT_LINE);		//取消右循迹标志位
					line_pid_param.kp = original_line_pid;  //恢复正常
					line_pid_param.outputMax =  max;
				}
				else										//差速转
				{	
					/*原地转*/
					if((nodesr.nowNode.flag&STOPTURN) || (fabsf(need2turn(nodesr.nowNode.angle,nodesr.nextNode.angle))>90))
					{
						/*原地转需要往前走一段 - 保证车在节点上再转*/
						motor_all.Gspeed = Stop_T_Speed;
						angle.AngleG = getAngleZ();
						pid_mode_switch(is_Gyro);
						if (nodesr.lastNode.nodenum == P8 && nodesr.nowNode.nodenum == C9 && nodesr.nextNode.nodenum == N22)
							Want2Go(28);
						else if (nodesr.lastNode.nodenum == S1 && nodesr.nowNode.nodenum == N3 && nodesr.nextNode.nodenum == P3)
							Want2Go(15);
						else if (nodesr.lastNode.nodenum == C8 && nodesr.nowNode.nodenum == C7 && nodesr.nextNode.nodenum == N14)
							Want2Go(23);
						else if (nodesr.lastNode.nodenum == C9 && nodesr.nowNode.nodenum == N22 && nodesr.nextNode.nodenum == B6)
							Want2Go(20);
						else if (nodesr.lastNode.nodenum == B3 && nodesr.nowNode.nodenum == N2 && nodesr.nextNode.nodenum == P2)
							Want2Go(30);
						else if (nodesr.lastNode.nodenum == B9 && nodesr.nowNode.nodenum == N7 && nodesr.nextNode.nodenum == P5)
							Want2Go(23);
						else if (nodesr.lastNode.nodenum == P5 && nodesr.nowNode.nodenum == N7 && nodesr.nextNode.nodenum == B8)
							Want2Go(25);
						else if (nodesr.lastNode.nodenum == P7 && nodesr.nowNode.nodenum == N20 && nodesr.nextNode.nodenum == C4)
							Want2Go(30);
						else if (nodesr.lastNode.nodenum == N8 && nodesr.nowNode.nodenum == N5 && nodesr.nextNode.nodenum == N4)
							Want2Go(40);
						else if (nodesr.lastNode.nodenum == N8 && nodesr.nowNode.nodenum == N3 && nodesr.nextNode.nodenum == P3)
							Want2Go(15);
						else if (nodesr.lastNode.nodenum == N8 && nodesr.nowNode.nodenum == N3 && nodesr.nextNode.nodenum == N4)
							Want2Go(30);
						else if (nodesr.lastNode.nodenum == N9 && nodesr.nowNode.nodenum == N10 && nodesr.nextNode.nodenum == N8)
							Want2Go(26);
						else if (nodesr.lastNode.nodenum == N20 && nodesr.nowNode.nodenum == C4 && nodesr.nextNode.nodenum == C8)
							Want2Go(25);
						else
							Want2Go(22);
						
						CarBrake();
						vTaskDelay(100);
						
						struct PID_param origin_param = gyroT_pid_param;
						char oriGmax = motor_all.GyroT_speedMax;

						if(nodesr.lastNode.nodenum == B3 && nodesr.nowNode.nodenum == N2 && nodesr.nextNode.nodenum == P2)
						{
							gyroT_pid_param.kp = 2.0; // 1.8
							gyroT_pid_param.ki = 0;
							gyroT_pid_param.kd = 20.0;
							motor_all.GyroG_speedMax = 9;
						}		
						else
						{
							motor_all.GyroT_speedMax = 20;
							gyroT_pid_param.kp = 2.0; // 1.8
							gyroT_pid_param.ki = 0;
							gyroT_pid_param.kd = 10.0;
						}

						angle.AngleT = nodesr.nextNode.angle;
						pid_mode_switch(is_Turn);
						while(fabs(need2turn(angle.AngleT,getAngleZ()))>2)
						{
							vTaskDelay(2);
							Cross_getline();
							if(Cross_Scaner.lineNum==1&&((Cross_Scaner.detail&0x180)!=0)&&(fabs(need2turn(angle.AngleT,getAngleZ()))<fabs(need2turn(angle.AngleT,nodesr.nowNode.angle))*0.15f))
								break;
						}

						motor_all.GyroT_speedMax = oriGmax;
						gyroT_pid_param = origin_param;
					}
					/*陀螺仪转*/
					else
					{
						/*参数调整*/
						struct PID_param origin_parm=gyroG_pid_param;
						float origin_speedMax = motor_all.GyroG_speedMax;
						
						gyroG_pid_param.kp = 2.0;
						gyroG_pid_param.ki = 0;
						gyroG_pid_param.kd = 22;

						if (nodesr.nowNode.nodenum == N1 || (nodesr.lastNode.nodenum == B2 && nodesr.nowNode.nodenum == N4 && nodesr.nextNode.nodenum == N5))
							motor_all.GyroG_speedMax = 25-3;
						else if (need2turn(nodesr.nowNode.angle, nodesr.nextNode.angle) > 0)
							motor_all.GyroG_speedMax = 30;
						else
							motor_all.GyroG_speedMax = 35;

						/*陀螺仪校正*/
						mpuZreset(imu.yaw, nodesr.nowNode.angle);

						/*前进一段距离*/
						Motor_Control(is_Gyro, Gyro_Speed, Gyro_Speed, nodesr.nowNode.angle);

						if (nodesr.lastNode.nodenum == B2 && nodesr.nowNode.nodenum == N4 && nodesr.nextNode.nodenum == N5)
							Want2Go(9);
						else if (nodesr.lastNode.nodenum == B3 && nodesr.nowNode.nodenum == N2 && nodesr.nextNode.nodenum == P2)
							Want2Go(15);
						else if (nodesr.lastNode.nodenum == P3 && nodesr.nowNode.nodenum == N3 && nodesr.nextNode.nodenum == N8)
							Want2Go(5);
						else if (nodesr.lastNode.nodenum == P8 && nodesr.nowNode.nodenum == C9 && nodesr.nextNode.nodenum == N22)
							Want2Go(3);
						else if (nodesr.lastNode.nodenum == S1 && nodesr.nowNode.nodenum == N3 && nodesr.nextNode.nodenum == N8)
							Want2Go(8);
						else if (nodesr.lastNode.nodenum == N3 && nodesr.nowNode.nodenum == N4 && nodesr.nextNode.nodenum == B3)
							Want2Go(15);
						else if (nodesr.lastNode.nodenum == N5 && nodesr.nowNode.nodenum == N12 && nodesr.nextNode.nodenum == N13)
							Want2Go(3);
						else if (nodesr.lastNode.nodenum == N5 && nodesr.nowNode.nodenum == N6 && nodesr.nextNode.nodenum == S2)
							Want2Go(12);
						else if (nodesr.lastNode.nodenum == N8 && nodesr.nowNode.nodenum == N12 && nodesr.nextNode.nodenum == N13)
							Want2Go(4);
						else if (nodesr.lastNode.nodenum == N8 && nodesr.nowNode.nodenum == N3 && nodesr.nextNode.nodenum == P3)
							Want2Go(20);
						else if (nodesr.lastNode.nodenum == N8 && nodesr.nowNode.nodenum == N3 && nodesr.nextNode.nodenum == S1)
							Want2Go(30);
						else if (nodesr.lastNode.nodenum == N9 && nodesr.nowNode.nodenum == N10 && nodesr.nextNode.nodenum == N8)
							Want2Go(4);
						else if (nodesr.lastNode.nodenum == N10 && nodesr.nowNode.nodenum == N3 && nodesr.nextNode.nodenum == S1)
							Want2Go(4);
						else if (nodesr.lastNode.nodenum == N15 && nodesr.nowNode.nodenum == N10 && nodesr.nextNode.nodenum == N8)
							Want2Go(5);

						/*转弯*/
						angle.AngleG = getAngleZ() + need2turn(nodesr.nowNode.angle, nodesr.nextNode.angle);
						if(angle.AngleG>180)
							angle.AngleG -= 360;
						else if(angle.AngleG<=-180)
							angle.AngleG += 360;
						while(fabsf(need2turn(getAngleZ(),angle.AngleG)) > 4)
						{
							vTaskDelay(2);
							Cross_getline();
							if(Cross_Scaner.lineNum == 1 && (Cross_Scaner.detail&0x3C0) && (fabs(need2turn(angle.AngleG,getAngleZ()))<fabs(need2turn(angle.AngleG,nodesr.nowNode.angle))*0.15f))
								break;
						}

						gyroG_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
						gyroG_pid_param=origin_parm;
						motor_all.GyroG_speedMax=origin_speedMax;
					}
				}
			}

			/*转完后进入循迹模式，更新结点，清空编码器值*/
			_flag=1;
			motor_all.CDOWNincrement = 0.6;
			half_times = 0;
			buzzer_off();
			pid_mode_switch(is_Line);
			nodesr.lastNode = nodesr.nowNode;
			nodesr.nowNode = nodesr.nextNode;
			motor_all.Cspeed = nodesr.nowNode.speed;
			nodesr.nextNode	= Node[getNextConnectNode(nodesr.nowNode.nodenum,route[map.point++])];
			
			if(nodesr.nowNode.nodenum == N9 && nodesr.nextNode.nodenum == N10)
				scaner_set.CatchsensorNum = line_weight[7];
			else
				scaner_set.CatchsensorNum = 0;
			
			scaner_set.EdgeIgnore = 0;
		    encoder_clear();
			LEFT_RIGHT_LINE = 0;
			mul2sing = 0;
			sing2mul = 0;
			nodesr.flag&=~0x04;		//	清除到达路口标志
		}
		else															//如果路线走完
		{	
			motor_all.Cspeed = 0;
			motor_all.Gspeed = 0;
			LEFT_RIGHT_LINE = 0;
			CarBrake();
			vTaskDelay(2);
			_flag=1;
			map.routetime+=1;
		}	
	}	

	/*看到红灯*/
	if(nodesr.flag&0x20)
	{		
		scaner_set.CatchsensorNum = 0;
		_flag=1;
		nodesr.lastNode = nodesr.nowNode;
		nodesr.nextNode	= Node[getNextConnectNode(nodesr.nowNode.nodenum,route[map.point++])];		//对下一结点进行更新
		nodesr.flag&=~0x20;
		motor_all.Cspeed = SPEED25;
		pid_mode_switch(is_Line);
	}

	/*看到非红灯*/
	if(nodesr.flag&0x80)		//如果打到门是开着
	{
		scaner_set.CatchsensorNum = 0;
		_flag=1;
		nodesr.nextNode	= Node[getNextConnectNode(nodesr.nowNode.nodenum,route[map.point++])];		//对下一结点进行更新
		nodesr.flag&=~0x80;
		encoder_clear();
		motor_all.Cspeed = SPEED3;
		pid_mode_switch(is_Line);
	}
}
	
/*函数选择*/
void map_function(u8 fun)
{
	switch(fun)
	{
		case 0:break;
		case 1:break;												//寻线
		case UpStage      :Stage();					break;			//平台
		case Bridge  	  :Barrier_Bridge();		break;			//长桥
		case Hill	  	  :Barrier_Hill();			break;			//楼梯
		case SM           :Sword_Mountain();		break;			//刀山
		case View	      :view();					break;			//景点 后转
		case View1        :view1();					break;			//景点 直退
		case BACK         :back();					break; 
		case BSoutPole	  :South_Pole();			break;			//南极
		case QQB	      :QQB_1();					break;			//跷跷板
		case BLBS         :Barrier_WavedPlate(87);	break;			//短减速板 速度，长度 80//85
		case BLBL	      :Barrier_WavedPlate(160);	break;			//长减速板 速度，长度	//180
		case DOOR	      :door();					break;			//打门
		case BHM          :Barrier_HighMountain(Mount_Speed); break;	//上珠峰
		case IGNORE       :ignore_node(); 			break;  		//忽略该节点
		case UNDER        :undermou();				break;
		case Special_node :Special_Node();			break;
		case UpStageP2	  :Stage_P2();				break;
		default:									break;		
	}
}
