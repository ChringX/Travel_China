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
#include "gray.h"
#include "string.h"

#define LINE_SPEED_MAX 100
#define LINEG_SPEED_MAX 100
#define Speed_Compensate 5
#define BLACK 0	 // 循黑线
#define WHLITE 1 // 循白线

volatile uint8_t LEFT_RIGHT_LINE = 0;
float Fspeed;				//经过PID运算后的结果
const float line_weight_default[16] = {-3, -2.4, -1.8, -1.3, -0.9, -0.6, -0.4, -0.2, 0.2, 0.4, 0.6, 0.9, 1.3, 1.8, 2.4, 3};
const float lineG_weight_default[16] = {3, 2.4, 1.8, 1.3, 0.9, 0.6, 0.4, 0.2, -0.2, -0.4, -0.6, -0.9, -1.3, -1.8, -2.4, -3};
float line_weight[16];		//激光从左到右各灯权重
volatile struct Scaner_Set scaner_set = {0, 0};
volatile SCANER Scaner;
volatile SCANER Cross_Scaner;
#define Line_color WHLITE



uint8_t isFilter = 1;
/*循迹PID计算*/
void Go_Line(float speed)
{
	if(isFilter)
	{
		line_pid_obj.measure = Get_scaner_error();
		// printf("mea:%.2f\r\n", line_pid_obj.measure);
	}
	else
		line_pid_obj.measure = Scaner.error;					// 当前循迹板所在的位置，从左到右-7到0到0到7
	line_pid_obj.target = scaner_set.CatchsensorNum; 			//目标

	Fspeed = positional_PID(&line_pid_obj, &line_pid_param);
	if (Fspeed >= LINE_SPEED_MAX)
		Fspeed = LINE_SPEED_MAX;
	else if (Fspeed <= -LINE_SPEED_MAX)
		Fspeed = -LINE_SPEED_MAX;
		
	Fspeed *= fabsf(speed) / 50;

	motor_all.Lspeed = speed - Fspeed;
	motor_all.Rspeed = speed + Fspeed;
}

/*获取模式处理后的循迹值*/
uint8_t getline_error(void)
{
	get_detail();
	Line_Scan(&Scaner, Lamp_Max, scaner_set.EdgeIgnore);
	return 0;
}

/*节点间临时循迹值获取*/
void Cross_getline(void)
{
	u8 linenum = 0; // 记录线的数目
	u8 lednum = 0;
	uint16_t data = 0XFFFF;

	data ^= ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14)) <<15);
	data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))	<<14);
	data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))	<<13);
	data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4))	<<12);
	data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))	<<11);
	data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7))	<<10);
	data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))	<<9);
	data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)) <<8);
	data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5))	<<7);
	data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0))	<<6);	
	data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4))	<<5);
	data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3))	<<4);
	data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3))	<<3);
	data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2))	<<2);
	data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1))	<<1);
	data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14))	<<0); //不同输出1.相同输出

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

/*获取各灯值*/
void get_detail(void)
{
	uint16_t data;
	if (ScanerMode == RF)
	{
		data = 0XFFFF;
		data ^= ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14)) << 15);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)) << 14);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)) << 13);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)) << 12);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)) << 11);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7)) << 10);
		data ^= ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)) << 9);
		data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)) << 8);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5)) << 7);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0)) << 6);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4)) << 5);
		data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)) << 4);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3)) << 3);
		data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)) << 2);
		data ^= ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1)) << 1);
		data ^= ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)) << 0); // 不同输出1.相同输出
	}
	else if(ScanerMode == Gray)
	{
		data = Gray_GetLine();
	}

	Scaner.detail = data;
}

struct Line_data // 循迹速度结构体
{
	volatile float pos;		// 灯的中心位置
	volatile float error;	// 误差
	volatile uint8_t truth; // 该值是否正确
} line_data[5] = {
	{0.0f, 0.0f, 1}, // 第一个结构体初值
	{0.0f, 0.0f, 1}, // 第二个结构体初值
	{0.0f, 0.0f, 1}, // 第三个结构体初值
	{0.0f, 0.0f, 1}, // 第四个结构体初值
	{0.0f, 0.0f, 1}	 // 第五个结构体初值
};
enum
{
	NO_error,
	all_error,
	pos_error
};
/*循线扫描 - 包括各种模式处理*/
uint8_t Line_Scan(volatile SCANER *scaner, unsigned char sensorNum, int8_t edge_ignore)
{
	float error = 0;
	u8 linenum = 0; 	//线数
	u8 lednum = 0;		//灯数
	uint8_t lednum_tmp = 0;

	/*获取二进制循迹值*/
	for (uint8_t i = 0; i < sensorNum; i++) 	// 从小车方向从左往右数亮灯数和引导线数
	{											// linenum用来记录有多少条线，line用来记录第几条线。
		if ((scaner->detail & (0x1 << i)))
		{
			lednum++;
			if (!(scaner->detail & (1 << (i + 1))))
				++linenum; 						// 先读取亮灯数和引导线数，检测到从1变为0认为一条线
		}
	}
	scaner->lineNum = linenum;
	scaner->ledNum = lednum;

	/*粗略检测 - 滤掉必定错误的值*/
	if (error_detect_one(lednum, linenum))
	{
		Update_line_data(all_error, -1, -1);
		return 0;
	}

	/*循迹中心值计算 - error值计算*/
	float pos = value_calculation(scaner, edge_ignore, sensorNum, &error, &lednum_tmp);

	/*位置正确性判断*/
	if (pos >= 0)
	{
		if (pos_detect(pos)) // 正确
		{
			error /= (float)lednum_tmp; // 取平均
			scaner->error = error;
			Update_line_data(NO_error, pos, scaner->error);
		}
		else
		{
			error /= (float)lednum_tmp; // 取平均
			scaner->error = error;
			Update_line_data(pos_error, pos, scaner->error);
		}
	}
	else
	{
		Update_line_data(all_error, -1, -1);
	}
	return 0;
}

/*打印出u16变量的二进制值 - 前半为二进制值，后半为原始数据*/
void printf_byte(uint16_t data)
{
    /*打印二进制值*/
    for(int16_t i=sizeof(data)*8-1; i>=0; i--)
    {
        printf("%d", (data>>i)&1);
    }
    printf("\t%d\r\n",data);
}



/*循迹滤波*/
/*循迹中心值和位置计算 - 正确返回大于等于0的位置，错误返回负数*/
#define MAX_LED	4
float value_calculation(volatile SCANER *scaner, int8_t edge_ignore, unsigned char SensorNum, float *Error, u8 *LED_Num_Temp)
{
	float pos = 0;
	/*获取需要的循迹值 - 以L_R_open为最高循迹优先级*/
	if (LEFT_RIGHT_LINE != 0)
	{
		/*左循线*/
		if (LEFT_RIGHT_LINE == 1)
		{
			for (uint8_t i = edge_ignore; i < SensorNum - edge_ignore; i++)
			{
				*LED_Num_Temp += (scaner->detail >> (SensorNum - 1 - i)) & 0X01;
				*Error += ((scaner->detail >> (SensorNum - 1 - i)) & 0X01) * line_weight[i];
				if ((scaner->detail >> (SensorNum - 1 - i)) & 0X01)
					pos += i;
				/*如果是白线*/
				if ((scaner->detail >> (SensorNum - i - 1)) & 0X01)
				{
					if (!((scaner->detail >> ((SensorNum - i - 1) - 1)) & 0x01)) // 下一个灯不是白
					{
						break; // 退出
					}
				}
			}
			if ((*LED_Num_Temp > MAX_LED)) // 目标灯数过多  引导线过多
			{
				return -1;
			}
		}
		/*右巡线*/
		else if (LEFT_RIGHT_LINE == 2)
		{
			for (uint8_t i = edge_ignore; i < SensorNum - edge_ignore; i++)
			{
				*LED_Num_Temp += (scaner->detail >> i) & 0X01;
				*Error += ((scaner->detail >> i) & 0X01) * line_weight[SensorNum - 1 - i];
				if ((scaner->detail >> i) & 0X01)
					pos += SensorNum - 1 - i;
				if ((scaner->detail >> i) & 0X01)
				{
					if (!((scaner->detail >> (i + 1)) & 0x01))
					{
						break;
					}
				}
			}
			if ((*LED_Num_Temp > MAX_LED)) // 目标灯数过多  引导线过多
			{
				return -1;
			}
		}
		/*居中流水*/
		else if (LEFT_RIGHT_LINE == 3)
		{
			float best_location = 0.0f;
			float temp_location = 0.0f;
			uint8_t line_led_last = 0;
			uint8_t len = 0;
			uint8_t temp_len = 0;

			for (uint8_t i = edge_ignore; i < SensorNum - edge_ignore; i++)
			{
				if ((scaner->detail & (1 << i)))
				{
					temp_location += i;
					temp_len++;
					if (!(scaner->detail & (1 << (i + 1))))
					{
						temp_location /= (float)temp_len; // 获取平均位置
						if (fabs(temp_location - (((float)(SensorNum - 1)) / 2)) < fabs(best_location - (((float)(SensorNum - 1)) / 2)))
						{
							best_location = temp_location;
							line_led_last = i; // 保存
							len = temp_len;	   // 保存
							temp_location = 0;
							temp_len = 0;
						}
					}
				}
			}
			for (uint8_t i = line_led_last - len + 1; i <= line_led_last; i++)
			{
				*LED_Num_Temp += (scaner->detail >> i) & 1;
				*Error += ((scaner->detail >> i) & 1) * line_weight[SensorNum - 1 - i];
				if ((scaner->detail >> i) & 1)
					pos += SensorNum - 1 - i;
			}
			if ((*LED_Num_Temp > MAX_LED)) // 目标灯数过多  引导线过多
			{
				return -1;
			}
		}
	}
	else
	{
		/*左循线 - LEFT_LINE*/
		if ((nodesr.nowNode.flag & LEFT_LINE) == LEFT_LINE)
		{
			for (uint8_t i = edge_ignore; i < SensorNum - edge_ignore; i++)
			{
				*LED_Num_Temp += (scaner->detail >> (SensorNum - 1 - i)) & 0X01;
				*Error += ((scaner->detail >> (SensorNum - 1 - i)) & 0X01) * line_weight[i];
				if ((scaner->detail >> (SensorNum - 1 - i)) & 0X01)
					pos += i;
				if ((scaner->detail >> (SensorNum - i - 1)) & 0X01)				 // 如果是白线
					if (!((scaner->detail >> ((SensorNum - i - 1) - 1)) & 0x01)) // 下一个灯不是白
						break;														 // 退出
			}
			if ((*LED_Num_Temp > MAX_LED)) // 目标灯数过多  引导线过多
			{
				return -1;
			}
		}
		/*右循线 - RIGHT_LINE*/
		else if ((nodesr.nowNode.flag & RIGHT_LINE) == RIGHT_LINE)
		{
			for (uint8_t i = edge_ignore; i < SensorNum - edge_ignore; i++)
			{
				*LED_Num_Temp += (scaner->detail >> i) & 0X01;
				*Error += ((scaner->detail >> i) & 0X01) * line_weight[SensorNum - 1 - i];
				if ((scaner->detail >> i) & 0X01)
					pos += SensorNum - 1 - i;
				if ((scaner->detail >> i) & 0X01)
					if (!((scaner->detail >> (i + 1)) & 0x01))
						break;
			}

			if ((*LED_Num_Temp > MAX_LED)) // 目标灯数过多  引导线过多
			{
				return -1;
			}
		}
		/*居中流水 - M_GO*/
		else if ((nodesr.nowNode.flag & LiuShui) == LiuShui)
		{
			float best_location = 0.0f;
			float temp_location = 0.0f;
			uint8_t line_led_last = 0;
			uint8_t len = 0;
			uint8_t temp_len = 0;
			for (uint8_t i = edge_ignore; i < SensorNum - edge_ignore; i++)
			{
				if ((scaner->detail & (0x1 << i)))
				{
					temp_location += i; //
					temp_len++;
					if (!(scaner->detail & (1 << (i + 1))))
					{
						temp_location /= (float)temp_len; // 获取平均位置
						if (fabs(temp_location - (((float)(SensorNum - 1)) / 2)) < fabs(best_location - (((float)(SensorNum - 1)) / 2)))
						{
							best_location = temp_location;
							line_led_last = i; // 保存
							len = temp_len;	   // 保存
							temp_location = 0;
							temp_len = 0;
						}
					}
				}
			}
			for (uint8_t i = line_led_last - len + 1; i <= line_led_last; i++)
			{
				*LED_Num_Temp += (scaner->detail >> i) & 0X01;
				*Error += ((scaner->detail >> i) & 0X01) * line_weight[SensorNum - 1 - i];
				if ((scaner->detail >> i) & 0X01)
					pos += SensorNum - 1 - i;
			}
			if ((*LED_Num_Temp > MAX_LED)) // 目标灯数过多  引导线过多
			{
				return -1;
			}
		}

		/*其他*/
		else
		{
			if (scaner->ledNum >= 4 && scaner->lineNum >= 2)
			{
				edge_ignore = 4;
			}
			for (uint8_t i = edge_ignore; i < SensorNum - edge_ignore; i++)
			{
				*LED_Num_Temp += (scaner->detail >> (SensorNum - 1 - i)) & 0X01;
				*Error += ((scaner->detail >> (SensorNum - 1 - i)) & 0X01) * line_weight[i];
				if ((scaner->detail >> (SensorNum - 1 - i)) & 0X01)
					pos += i;
			}
			if ((*LED_Num_Temp > MAX_LED)) // 目标灯数过多  引导线过多
			{
				return -1;
			}
		}
	}

	pos /= (float)(*LED_Num_Temp);
	return pos;
}
/*更新循迹值数组 - 错误类型 0为无错，为正确值 1为精检验错误  2为粗略检测错误*/
void Update_line_data(uint8_t error_kind, float pos, float error)
{
	memmove(&line_data, &line_data[1], sizeof(struct Line_data) * 4); // 递推平均滤波法
	switch (error_kind)
	{
	case NO_error: // 无错误
		line_data[4].error = error;
		line_data[4].pos = pos;
		line_data[4].truth = error_kind;
		break;
	case all_error: // 全错误
		line_data[4].error = -1;
		line_data[4].pos = -1;
		line_data[4].truth = error_kind;
		break;
	case pos_error:
		line_data[4].error = error;
		line_data[4].pos = pos;
		line_data[4].truth = error_kind;
		break;
	default:
		break;
	}
}


#define pos_max_error 1.5f
/*判断位置和上一次正确值是否相近 - 正确则返回1 错误返回0*/
uint8_t pos_detect(float pos)
{
	uint8_t flag = 0;
	uint8_t idx = 0;
	// 找到最近的正确值
	for (int i = 4; i >= 0; i--)
	{
		if (line_data[i].truth == 0)
		{
			idx = i;
			flag = 1;
			break;
		}
	}
	// 有正确值对比
	if (flag)
	{
		if (fabs(line_data[idx].pos - pos) < pos_max_error)
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	// 无正确值对比  只能认为这个是对的咯
	else
	{
		return 1;
	}
}
#define R_ 1
/*获取循迹值error*/
float Get_scaner_error(void)
{
	uint8_t nums = 0;
	float error = 0;
	float pos_data[5] = {0, 0, 0, 0, 0};
	float pos_pos[5] = {0, 0, 0, 0, 0};
	uint8_t pos_error_nums = 0;
	uint8_t idex[5] = {0, 0, 0, 0, 0};

	/*判断正确数据多还是错误数据多*/
	for (int i = 0; i < 5; i++)
	{
		if (line_data[i].truth == 0)
		{
			idex[nums] = i;
			nums++; // 数据正确的数量
		}
		else if (line_data[i].truth == pos_error)
		{
			pos_data[pos_error_nums] = line_data[i].error;
			pos_pos[pos_error_nums] = line_data[i].pos;
			pos_error_nums++; // pos错误的数据数量
		}
	}

	if (nums >= 3 || (nums >= pos_error_nums && nums != 0)) // 正确的数量多，并且正确数量不为0
	{
		// printf("代码路过区域\n");
		for (int i = 0; i < nums; i++)
		{
			error += line_data[idex[i]].error;
		}
		error /= (float)nums;
	}
	else if (nums == 0) // 没有正确值
	{
		if (pos_error_nums == 0) // 没有pos值可参考
		{
			return 0; // 没有办法，硬着头皮原路前进
		}
		else // 参考错误的位置或许可行
		{
			if (pos_error_nums == 1) // 只有一个可参考
			{
				return pos_data[0];
			}

			/*奖励机制*/
			uint8_t scorce[5] = {0, 0, 0, 0, 0};
			for (int i = 0; i < pos_error_nums; i++)
			{
				for (int j = i + 1; j < pos_error_nums; j++)
				{
					if (fabs(pos_pos[i] - pos_pos[j]) <= R_)
					{
						scorce[i]++;
						scorce[j]++;
					}
				}
			}

			/*找出最多临近的位置*/
			uint8_t max = scorce[0]; // 假设第一个元素是最大的
			uint8_t max_idx = 0;
			for (int i = 0; i < pos_error_nums; i++)
			{
				if (max < scorce[i])
				{
					max = scorce[i];
					max_idx = i;
				}
			}
			/*都是离散分散的，没救了*/
			if (max == 0)
			{
				return 0;
			}
			else
			{
				return pos_data[max_idx];
			}
		}
	}
	else // 正确值比错误值少 需要判断有效错误值是否大于正确值
	{
		/*奖励机制*/
		uint8_t scorce[5] = {0, 0, 0, 0, 0};
		for (int i = 0; i < pos_error_nums; i++)
		{
			for (int j = i + 1; j < pos_error_nums; j++)
			{
				if (fabs(pos_pos[i] - pos_pos[j]) <= R_)
				{
					scorce[i]++;
					scorce[j]++;
				}
			}
		}
		/*找出最多临近的位置*/
		uint8_t max = scorce[0]; // 假设第一个元素是最大的
		uint8_t max_idx = 0;
		for (int i = 0; i < pos_error_nums; i++)
		{
			if (max < scorce[i])
			{
				max = scorce[i];
				max_idx = i;
			}
		}
		if (nums >= max + 1) // 正确值偏多，这里的加一是因为奖励机制中没把自己也当好朋友算进去
		{
			for (int i = 0; i < nums; i++)
			{
				error += line_data[idex[i]].error;
			}
			error /= (float)nums;
		}
		else
		{
			return pos_data[max_idx];
		}
	}
	return error;
}

/*粗略检测 - 判断该循迹值是否可用，可用返回1，不可用返回0*/
uint8_t error_detect_one(u8 LED_Num, u8 Line_Num)
{
	// 多灯 多线 无灯 灯数/线数 >=4
	if (LED_Num >= 10 || Line_Num >= 4 || LED_Num == 0 || LED_Num / Line_Num >= 4)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
