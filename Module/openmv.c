/*
 * @File: openmv.c
 * @Description:7A4 8A4 9A3 10B6 11A2 12B5 13B6  14A1  15B3  16A0  17B4  18B2  19B1  20A6  21B0  22A5  23黄 24六    25五    26四    27三     28二     29直立  30准备   31绿   32宝物  33红  34七  35路口  36八号 
 * @Version: 1.0.0
 * @Author:
 * @Date: 2023-09-13 20:33:36
 * @LastEditTime: 2023-09-15 16:02:18
 */
//16A0  14A1  11A2  9A3   8A4   22A5  20A6         
//21B0  19B1  18B2  15B3  17B4  12B5  10B6  13B6  
//28二    27三    26四    25五   24六   34七    36八号  29直立 
//23黄    31绿    33红           
//30准备  32宝物  35路口    


 
#include "openmv.h"
#include "usart.h"
#include "math.h"
#include "stdio.h"
#include "uart.h"
UART_HandleTypeDef mv;	 // UART句柄
UART_HandleTypeDef mv_R; // UART句柄
uint8_t color;
uint8_t color_R;

/**
 * @brief:
 * @param {uint32_t} bound
 * @return {*}
 */
void mv_init(uint32_t bound)
{
	// UART 初始化设置
	mv.Instance = UART4;
	mv.Init.BaudRate = bound;				 // 波特率
	mv.Init.WordLength = UART_WORDLENGTH_8B; // 字长为8位数据格式
	mv.Init.StopBits = UART_STOPBITS_1;		 // 一个停止位
	mv.Init.Parity = UART_PARITY_NONE;		 // 无奇偶校验位
	mv.Init.HwFlowCtl = UART_HWCONTROL_NONE; // 无硬件流控
	mv.Init.Mode = UART_MODE_TX_RX;			 // 收发模式
	HAL_UART_Init(&mv);						 // HAL_UART_Init()会使能UART3
	__HAL_UART_ENABLE_IT(&mv, UART_IT_RXNE);
	close_mv();
}

/**
 * @brief:
 * @param {uint32_t} bound
 * @return {*}
 */
void mvR_init(uint32_t bound)
{
	// UART 初始化设置
	mv_R.Instance = USART6;
	mv_R.Init.BaudRate = bound;				   // 波特率
	mv_R.Init.WordLength = UART_WORDLENGTH_8B; // 字长为8位数据格式
	mv_R.Init.StopBits = UART_STOPBITS_1;	   // 一个停止位
	mv_R.Init.Parity = UART_PARITY_NONE;	   // 无奇偶校验位
	mv_R.Init.HwFlowCtl = UART_HWCONTROL_NONE; // 无硬件流控
	mv_R.Init.Mode = UART_MODE_TX_RX;		   // 收发模式
	HAL_UART_Init(&mv_R);					   // HAL_UART_Init()会使能UART3
	__HAL_UART_ENABLE_IT(&mv_R, UART_IT_RXNE);
	//	open_mvR()
	close_mvR();
}

/**
 * @brief:
 * @return {*}
 */
void open_mv()
{
	uint8_t data = 0x55;
	HAL_UART_Transmit(&mv, &data, 1, 0xffff);
}

/**
 * @brief:
 * @return {*}
 */
void close_mv()
{
	color = 0;
	uint8_t data = 0x66;
	HAL_UART_Transmit(&mv, &data, 1, 0xffff);
}

/**
 * @brief:
 * @return {*}
 */
void open_mvR()
{
	uint8_t data = 0x55;
	HAL_UART_Transmit(&mv_R, &data, 1, 0xffff);
}

/**
 * @brief:
 * @return {*}
 */
void close_mvR()
{
	color_R = 0;
	uint8_t data = 0x66;
	HAL_UART_Transmit(&mv_R, &data, 1, 0xffff);
}

/**
 * @brief: 串口4中断处理，做了什么事情
 * @return {*}
 */
void UART4_IRQHandler(void)
{
	static uint8_t temp = 0;
	static uint8_t flag = 0;
	static uint8_t tempcolor[5] = {0,0,0,0,0};
	static uint8_t i = 0;
	if (__HAL_UART_GET_FLAG(&mv, UART_FLAG_RXNE) != 0)
	{
		temp = mv.Instance->RDR;

		if (flag == 2 && temp != 0xff && temp != 0x0a)
		{
			tempcolor[i++] = temp;
			flag = 0;
			if((tempcolor[0] == tempcolor[1] == tempcolor[2] == tempcolor[3] == tempcolor[4]) && tempcolor[0] != 0)
			{
				color = tempcolor[0];
				tempcolor[0] = tempcolor[1] = tempcolor[2] = tempcolor[3] = tempcolor[4] = 0;
				i = 0;
			}
			if(i > 4)
			{
				i = 0;
			}
		}
		if (temp == 0xff && flag == 1)
		{
			flag = 2;
		}
		if (temp == 0xff && flag == 0)
		{
			flag = 1;
		}
	}
	mv.Instance->ISR = 0; // 清除SR标志位
	HAL_UART_IRQHandler(&mv);
}

/**
 * @brief: 串口6中断处理，做了什么事情
 * @return {*}
 */
void USART6_IRQHandler(void)
{
	static uint8_t temp = 0;
	static uint8_t flag = 0;
	static uint8_t tempcolor[5] = {0,0,0,0,0};
	static uint8_t i = 0;
	if (__HAL_UART_GET_FLAG(&mv_R, UART_FLAG_RXNE) != 0)
	{
		temp = mv_R.Instance->RDR;

		if (flag == 2 && temp != 0xff && temp != 0x0a)
		{
			tempcolor[i++] = temp;
			flag = 0;
			if((tempcolor[0] == tempcolor[1] == tempcolor[2] == tempcolor[3] == tempcolor[4]) && tempcolor[0] != 0)
			{
				color = tempcolor[0];
				tempcolor[0] = tempcolor[1] = tempcolor[2] = tempcolor[3] = tempcolor[4] = 0;
				i = 0;
			}
			if(i > 4)
			{
				i = 0;
			}
		}
		if (temp == 0xff && flag == 1)
		{
			flag = 2;
		}
		if (temp == 0xff && flag == 0)
		{
			flag = 1;
		}
	}
	mv_R.Instance->ISR = 0; // 清除SR标志位
	HAL_UART_IRQHandler(&mv_R);
}

/**
 * @brief: 播放歌曲
 * @return {*}
 */
void send_play_command(void)
{
	uint8_t data[5] = {0x7e, 0x03, 0x01, 0x02, 0xef};
	HAL_UART_Transmit(&huart8, data, 5, 0xFFFF);
}

/**
 * @brief: 停止播放歌曲
 * @return {*}
 */
void send_stop_play_command(void)
{
	uint8_t data[5] = {0x7e, 0x03, 0x02, 0x01, 0xef};
	HAL_UART_Transmit(&huart8, data, 5, 0xFFFF);
}

/**
 * @brief: 播放指定歌曲
 * @param {uint8_t} index 第几首歌曲
 * @return {*}
 */
void send_play_specified_command(uint8_t index)
{
	// 7E 05 41 00(歌曲高位) 01(歌曲低位) 45(校验和) EF
	uint8_t data[7] = {0x7e, 0x05, 0x41, 0x00, 0x00, 0x00, 0xef};
	data[4] = index;
	uint8_t sum = data[1] ^ data[2] ^ data[3] ^ data[4];
	data[5] = sum;
	for (uint8_t i = 0; i < 7; i++)
	{
		HAL_UART_Transmit(&huart8, &data[i], 1, 0xFFFF);
	}
}
