/*
 * @File: scaner.c
 * @Description: 
 * @Version: 1.0.0
 * @Author: 
 * @Date: 2023-09-13 20:33:34
 * @LastEditTime: 2023-09-15 15:19:59
 */
#include "scaner.h"
#include "map.h"
#include "math.h"
#include "turn.h"
#include "stdio.h"
#include "pid.h"
#include "motor_task.h"
#include "speed_ctrl.h"
#include "bsp_linefollower.h"
#include "motor.h"

#define LINE_SPEED_MAX 100
#define Speed_Compensate 5
#define BLACK 0	 // 循黑线
#define WHLITE 1 // 循白线

volatile uint8_t LEFT_RIGHT_LINE = 0;
float Fspeed;																										// 经过PID运算后的结果
float line_weight[16] = {-3, -2.4, -1.8, -1.3, -0.9, -0.6, -0.4, -0.175, 0.175, 0.4, 0.6, 0.9, 1.3, 1.8, 2.4, 3}; // 0.3
// const float line_weight[16] = {-2.7,-2.1,-1.4,-1.15,-0.8,-0.5,-0.3,-0.15,0.15,0.3,0.5,0.8,1.15,1.4,2.1,2.7};//0.3
// const float line_weight[16] = {-3.1,-2.5,-2,-1.4,-0.9,-0.4,-0.2,-0.1,0.1,0.2,0.4,0.9,1.4,2,2.5,3.1};
// 从右到左//中间本来是0.1  外面是3.1
volatile struct Scaner_Set scaner_set = {0, 0};

// float CatchsensorNum = 0;	//寻迹时的目标传感器位置，一般情况下为0，微微会用到
volatile SCANER Scaner;
volatile SCANER Cross_Scaner;
#define Line_color WHLITE
// 循迹板 1234578 87654321

/**
 * @brief:
 * @param {float} speed
 * @return {*}
 */
void Go_Line(float speed)
{

	line_pid_obj.measure = Scaner.error;			 // 当前循迹板所在的位置，从左到右-7到0到0到7
	line_pid_obj.target = scaner_set.CatchsensorNum; // 目标
	//	printf("error=%f\r\n",line_pid_obj.measure);
	Fspeed = positional_PID(&line_pid_obj, &line_pid_param); // 进行位置PID运算

	if (Fspeed >= LINE_SPEED_MAX)
		Fspeed = LINE_SPEED_MAX;
	else if (Fspeed <= -LINE_SPEED_MAX)
		Fspeed = -LINE_SPEED_MAX;

	Fspeed *= fabsf(speed) / 50;
	//	printf("%f\r\n",Fspeed);

	motor_all.Lspeed = speed - Fspeed;
	motor_all.Rspeed = speed + Fspeed;
}

/**
 * @brief: 获得更新不同巡线模式下的误差值
 * @return {*}
 */
uint8_t getline_error(void)
{
	get_detail(); // 获取巡线值
	if (Line_Scan(&Scaner, Lamp_Max, scaner_set.EdgeIgnore))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void Cross_getline(void)
{
	u8 linenum = 0; // 记录线的数目
	u8 lednum = 0;
	uint16_t data = 0XFFFF;
	if(ScanMode == is_Front)
	{						// 白线
		data ^= ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14))<<15);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))	<<14);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))	<<13);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4))	<<12);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))	<<11);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7))	<<10);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))	<<9);
		data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15))<<8);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5))	<<7);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0))	<<6);	
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4))	<<5);
		data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3))	<<4);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3))	<<3);
		data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2))	<<2);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1))	<<1);
		data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14))	<<0); //不同输出1.相同输出
	}
	else
	{
		data ^= ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14))	<<0);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))	<<1);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))	<<2);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4))	<<3);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))	<<4);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7))	<<5);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))	<<6);
		data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15))	<<7);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5))	<<8);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0))	<<9);	
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4))	<<10);
		data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3))	<<11);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3))	<<12);
		data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2))	<<13);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1))	<<14);
		data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14))	<<15); //不同输出1.相同输出0
	}
	Cross_Scaner.detail = data;
	for (uint8_t i = 0; i < 16; i++) // 从小车方向从左往右数亮灯数和引导线数
	{										// linenum用来记录有多少条线，line用来记录第几条线。
		if (Cross_Scaner.detail & (0x1 << i))
		{
			lednum++;
			if (!(Cross_Scaner.detail & (1 << (i + 1))))
				linenum++; // 先读取亮灯数和引导线数，检测到从1变为0认为一条线
		}
	}
	Cross_Scaner.lineNum = linenum;
	Cross_Scaner.ledNum = lednum;
}

/**
 * @brief:
 * @return {*}
 */
void get_detail(void)
{
	uint16_t data = 0XFFFF;
	if(ScanMode == is_Front)
	{						// 白线
		data ^= ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14))<<15);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))	<<14);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))	<<13);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4))	<<12);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))	<<11);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7))	<<10);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))	<<9);
		data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15))<<8);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5))	<<7);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0))	<<6);	
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4))	<<5);
		data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3))	<<4);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3))	<<3);
		data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2))	<<2);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1))	<<1);
		data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14))	<<0); //不同输出1.相同输出
	}
	else
	{
		data ^= ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14))	<<0);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))	<<1);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))	<<2);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4))	<<3);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))	<<4);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7))	<<5);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))	<<6);
		data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15))	<<7);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5))	<<8);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0))	<<9);	
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4))	<<10);
		data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3))	<<11);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3))	<<12);
		data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2))	<<13);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1))	<<14);
		data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14))	<<15); //不同输出1.相同输出0
	}
	Scaner.detail = data;
}

/**
 * @brief: 循迹扫描
 * @param {SCANER} *scaner
 * @param {unsigned char} sensorNum
 * @param {int8_t} edge_ignore
 * @return {*}
 */
uint8_t Line_Scan(volatile SCANER *scaner, unsigned char sensorNum, int8_t edge_ignore)
{
	float error = 0;
	u8 linenum = 0; // 记录线的数目
	u8 lednum = 0;
	int8_t lednum_tmp = 0;
	// 获得二进制巡线值
	for (uint8_t i = 0; i < sensorNum; i++) // 从小车方向从左往右数亮灯数和引导线数
	{										// linenum用来记录有多少条线，line用来记录第几条线。
		if ((scaner->detail & (0x1 << i)))
		{
			lednum++;
			if (!(scaner->detail & (1 << (i + 1))))
				++linenum; // 先读取亮灯数和引导线数，检测到从1变为0认为一条线
		}
	}
	scaner->lineNum = linenum;
	scaner->ledNum = lednum;

	if (LEFT_RIGHT_LINE != 0)
	{
		// 左循迹
		if (LEFT_RIGHT_LINE == 1)
		{
			for (uint8_t i = edge_ignore; i < sensorNum - edge_ignore; i++)
			{
				lednum_tmp += (scaner->detail >> (sensorNum - i - 1)) & 0X01; // 记录点灯数
				error += ((scaner->detail >> (sensorNum - i - 1)) & 0X01) * line_weight[i];
				if ((scaner->detail >> (sensorNum - i - 1)) & 0X01)			 // 如果是白线
					if (!((scaner->detail >> ((sensorNum - i - 2))) & 0x01)) // 下一个灯不是白
						break;												 // 退出
			}
			if (lednum_tmp > 5)
			{
				Scaner.error = 0;
				return 0;
			}
		}
		// 右循迹
		else if (LEFT_RIGHT_LINE == 2)
		{
			for (uint8_t i = edge_ignore; i < sensorNum - edge_ignore; i++)
			{
				lednum_tmp += (scaner->detail >> i) & 0X01;
				error += ((scaner->detail >> i) & 0X01) * line_weight[sensorNum - i - 1];
				if ((scaner->detail >> i) & 0X01)
					if (!((scaner->detail >> (i + 1)) & 0x01))
						break;
			}
			if (lednum_tmp > 5)
			{
				Scaner.error = 0;
				return 0;
			}
		}
		// 流水
		else if (LEFT_RIGHT_LINE == 3)
		{
			uint8_t Location_Temp = 0;
			uint8_t Length_Temp = 0;
			uint8_t Location_Best = 0;
			uint8_t Last_Line_Location = 0;
			uint8_t length = 0;
			for (uint8_t i = edge_ignore; i < sensorNum - edge_ignore; i++)
			{
				if ((scaner->detail >> i) & 0x01)
				{
					Location_Temp += i;
					Length_Temp++;
					if (!(scaner->detail >> (i + 1)) & 0x01)
					{
						Location_Temp /= (float)Length_Temp; // 平均位置
						if ((fabsf(Location_Temp - ((float)((sensorNum - 1) / 2)))) < (fabsf(Location_Best - ((float)((sensorNum - 1) / 2)))))
						{
							Location_Best = Location_Temp;
							Last_Line_Location = i;
							length = Length_Temp;
							Location_Temp = 0;
							Length_Temp = 0;
						}
					}
				}
			}
			for (uint8_t i = Last_Line_Location - length + 1; i <= Last_Line_Location; i++)
			{
				lednum_tmp += (scaner->detail >> i) & 0X01;
				error += ((scaner->detail >> i) & 0X01) * line_weight[sensorNum - i - 1];
			}
			if (lednum_tmp > 5)
			{
				Scaner.error = 0;
				return 0;
			}
		}
	}
	else if (LEFT_RIGHT_LINE == 0)
	{
		if ((nodesr.nowNode.flag & LEFT_LINE) == LEFT_LINE) // 左循线
		{
			for (uint8_t i = edge_ignore; i < sensorNum - edge_ignore; i++)
			{
				lednum_tmp += (scaner->detail >> (sensorNum - i - 1)) & 0X01; // 记录点灯数
				error += ((scaner->detail >> (sensorNum - i - 1)) & 0X01) * line_weight[i];
				if ((scaner->detail >> (sensorNum - i - 1)) & 0X01)			 // 如果是白线
					if (!((scaner->detail >> ((sensorNum - i - 2))) & 0x01)) // 下一个灯不是白
						break;												 // 退出
			}
			if (lednum_tmp > 5)
			{
				Scaner.error = 0;
				return 0;
			}
		}
		else if ((nodesr.nowNode.flag & RIGHT_LINE) == RIGHT_LINE) // 右循线
		{
			for (uint8_t i = edge_ignore; i < sensorNum - edge_ignore; i++)
			{
				lednum_tmp += (scaner->detail >> i) & 0X01;
				error += ((scaner->detail >> i) & 0X01) * line_weight[sensorNum - i - 1];
				if ((scaner->detail >> i) & 0X01)
					if (!((scaner->detail >> (i + 1)) & 0x01))
						break;
			}
			if (lednum_tmp > 5)
			{
				Scaner.error = 0;
				return 0;
			}
		}
		else if ((nodesr.nowNode.flag & INGNORE) == INGNORE) // 短直立景点后退
		{
			error = 0;
		}
		else if ((nodesr.nowNode.flag & LiuShui) == LiuShui)
		{
			uint8_t flag = 0;
			float error1 = 0;
			float error2 = 0;
			uint8_t lednum_temp1 = 0;
			uint8_t lednum_temp2 = 0;
			for (uint8_t i = 0; i < sensorNum; i++)
			{
				if (flag == 0) // 右边线
				{
					error1 += ((scaner->detail >> i) & 0X01) * line_weight[sensorNum - i - 1];
					lednum_temp1 += (scaner->detail >> i) & 0X01;
				}
				else if (flag == 1) // 左边线
				{
					error2 += ((scaner->detail >> i) & 0X01) * line_weight[sensorNum - i - 1];
					lednum_temp2 += (scaner->detail >> i) & 0X01;
				}
				if ((scaner->detail >> i) & 0X01)
				{
					if (!((scaner->detail >> (i + 1)) & 0x01))
					{
						flag = 1;
					}
				}
			}
			if (error1 != 0)
				error1 /= lednum_temp1;
			else
				error1 = 0;
			if (lednum_temp2 == 0) // 只有一条线
				error2 = 0;
			else
				error2 /= lednum_temp2;
			if (lednum >= 6)
			{
				Scaner.error = 0;
				return 0;
			}
			else
			{
				if (fabs(error2) > fabs(error1) || lednum_temp2 == 0)
				{
					Scaner.error = error1;
					return 0;
				}
				else if ((lednum_temp2 != 0) && (fabs(error2) < fabs(error1)))
				{
					Scaner.error = error2;
					return 0;
				}
			}
		}
		else
		{
			for (uint8_t i = edge_ignore; i < sensorNum - edge_ignore; i++)
			{
				lednum_tmp += (scaner->detail >> (sensorNum - 1 - i)) & 0X01;
				error += ((scaner->detail >> (sensorNum - 1 - i)) & 0X01) * line_weight[i];
			}
			if (scaner->ledNum >= 4 || lednum_tmp >= 4 || scaner->lineNum > 1)
			{
				Scaner.error = 0;
				return 0;
			}
		}
	}

	if (lednum == 0 || lednum_tmp == 0)
	{
		error = 0;
	}
	else
	{
		error /= (float)lednum_tmp; // 取平均
	}
	Scaner.error = error;
	return 0;
}


/*切换主导头*/
void MODE_Switch(int8_t MODE_need)
{
	if(MODE_need == is_Front)
	{
		ScanMode = is_Front;
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);		//给C8T6指令
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
	}
	else if(MODE_need == is_Back)
	{
		ScanMode = is_Back;
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
	}
	gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
	gyroG_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
	line_pid_obj = (struct P_pid_obj){0,0,0,0,0,0,0};
}

