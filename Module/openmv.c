#include "openmv.h"
#include "usart.h"
#include "math.h"
#include "stdio.h"
#include "uart.h"
#include "speed_ctrl.h"



UART_HandleTypeDef MV_Left, MV_Right;
uint8_t Color_Left, Color_Right;



/*初始化OpenMV*/
void MV_Init(uint32_t bound)
{
	/*右MV*/
	MV_Right.Instance = UART4;
	MV_Right.Init.BaudRate = bound;				 	 // 波特率
	MV_Right.Init.WordLength = UART_WORDLENGTH_8B; 	 // 字长为8位数据格式
	MV_Right.Init.StopBits = UART_STOPBITS_1;		 // 一个停止位
	MV_Right.Init.Parity = UART_PARITY_NONE;		 // 无奇偶校验位
	MV_Right.Init.HwFlowCtl = UART_HWCONTROL_NONE;   // 无硬件流控
	MV_Right.Init.Mode = UART_MODE_TX_RX;			 // 收发模式
	HAL_UART_Init(&MV_Right);						 // HAL_UART_Init()会使能UART3
	__HAL_UART_ENABLE_IT(&MV_Right, UART_IT_RXNE);
	// Close_MV_R();

	/*左MV*/
	MV_Left.Instance = USART6;
	MV_Left.Init.BaudRate = bound;				   	// 波特率
	MV_Left.Init.WordLength = UART_WORDLENGTH_8B; 	// 字长为8位数据格式
	MV_Left.Init.StopBits = UART_STOPBITS_1;	   	// 一个停止位
	MV_Left.Init.Parity = UART_PARITY_NONE;	   		// 无奇偶校验位
	MV_Left.Init.HwFlowCtl = UART_HWCONTROL_NONE; 	// 无硬件流控
	MV_Left.Init.Mode = UART_MODE_TX_RX;		   	// 收发模式
	HAL_UART_Init(&MV_Left);					   	// HAL_UART_Init()会使能UART3
	__HAL_UART_ENABLE_IT(&MV_Left, UART_IT_RXNE);
	// Close_MV_L();
}

/*打开右MV*/
void Open_MV_R()
{
	uint8_t data = 0x55;
	HAL_UART_Transmit(&MV_Right, &data, 1, 0xffff);
}

/*关闭右MV*/
void Close_MV_R()
{
	Color_Left = 0;
	uint8_t data = 0x66;
	HAL_UART_Transmit(&MV_Right, &data, 1, 0xffff);
}

/*打开左MV*/
void Open_MV_L()
{
	uint8_t data = 0x55;
	HAL_UART_Transmit(&MV_Left, &data, 1, 0xffff);
}

/*关闭左MV*/
void Close_MV_L()
{
	Color_Right = 0;
	uint8_t data = 0x66;
	HAL_UART_Transmit(&MV_Left, &data, 1, 0xffff);
}

/*右MV串口中断*/
void UART4_IRQHandler(void)
{
	static uint8_t temp = 0;
	static uint8_t flag = 0;
	static uint8_t tempcolor[5] = {0,0,0,0,0};
	static uint8_t i = 0;
	if (__HAL_UART_GET_FLAG(&MV_Right, UART_FLAG_RXNE) != 0)
	{
		temp = MV_Right.Instance->RDR;

		if (flag == 2 && temp != 0xff && temp != 0x0a)
		{
			tempcolor[i++] = temp;
			flag = 0;
			if((tempcolor[0] == tempcolor[1] == tempcolor[2] == tempcolor[3] == tempcolor[4]) && tempcolor[0] != 0)
			{
				Color_Right = tempcolor[0];
				tempcolor[0] = tempcolor[1] = tempcolor[2] = tempcolor[3] = tempcolor[4] = 0;
				i = 0;
			}
			if(i > 4)
				i = 0;
		}
		if (temp == 0xff && flag == 1)
			flag = 2;
		if (temp == 0xff && flag == 0)
			flag = 1;
	}
	MV_Right.Instance->ISR = 0; // 清除SR标志位
	HAL_UART_IRQHandler(&MV_Right);
}

/*左MV串口中断*/
void USART6_IRQHandler(void)
{
	static uint8_t temp = 0;
	static uint8_t flag = 0;
	static uint8_t tempcolor[5] = {0,0,0,0,0};
	static uint8_t i = 0;
	if (__HAL_UART_GET_FLAG(&MV_Left, UART_FLAG_RXNE) != 0)
	{
		temp = MV_Left.Instance->RDR;

		if (flag == 2 && temp != 0xff && temp != 0x0a)
		{
			tempcolor[i++] = temp;
			flag = 0;
			if((tempcolor[0] == tempcolor[1] == tempcolor[2] == tempcolor[3] == tempcolor[4]) && tempcolor[0] != 0)
			{
				Color_Left = tempcolor[0];
				tempcolor[0] = tempcolor[1] = tempcolor[2] = tempcolor[3] = tempcolor[4] = 0;
				i = 0;
			}
			if(i > 4)
				i = 0;
		}
		if (temp == 0xff && flag == 1)
			flag = 2;
		if (temp == 0xff && flag == 0)
			flag = 1;
	}
	MV_Left.Instance->ISR = 0; // 清除SR标志位
	HAL_UART_IRQHandler(&MV_Left);
}



/*
	1	六号
	2	五号
	3	四号
	4	三号
	5	二号
	6	直立
	7	准备
	8	绿
	9	宝物
	10	黄
	11	红
	12	七号
	13	路口
	14	八号
	15	错误
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
