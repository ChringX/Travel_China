#include "K210.h"
#include "stdio.h"
#include "barrier.h"
#include "usart.h"
#include "map.h"

uint8_t Clue_Num = {0};
uint8_t K210_Rece = 0;
uint8_t K210_RxTemp_L = 0;
uint8_t K210_RxTemp_R = 0;

/*使能K210*/
void K210_Enable(void)
{
	HAL_UART_Receive_IT(&huart2, &K210_RxTemp_L, 1);
	HAL_UART_Receive_IT(&huart5, &K210_RxTemp_R, 1);
}

/*打开K210*/
void open_K210(void)
{
	uint8_t data = 0x55;
	HAL_UART_Transmit(&huart2, &data, 1, 0xffff);
	HAL_UART_Transmit(&huart5, &data, 1, 0xffff);
}

/*关闭K210*/
void close_K210(void)
{
	uint8_t data = 0x66;
	HAL_UART_Transmit(&huart2, &data, 1, 0xffff);
	HAL_UART_Transmit(&huart5, &data, 1, 0xffff);
}

/*右K210接收中断*/
void UART5_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart5);

	static uint8_t flag = 0;
	if (K210_RxTemp_R == 0xff && flag == 0)
		flag = 1;
	else if (K210_RxTemp_R == 0xff && flag == 1)
		flag = 2;
	else if (flag == 2 && K210_RxTemp_R != 0xff && K210_RxTemp_R != 0x0a)
	{
		Clue_Num = K210_RxTemp_R;
		if ((nodesr.nowNode.nodenum == P1 && (K210_RxTemp_R == 3 || K210_RxTemp_R == 4)) ||
			(nodesr.nowNode.nodenum == P3 && (K210_RxTemp_R == 5 || K210_RxTemp_R == 6)) ||
		    (nodesr.nowNode.nodenum == P4 && (K210_RxTemp_R == 5 || K210_RxTemp_R == 6)) ||
			(nodesr.nowNode.nodenum == P5 && (K210_RxTemp_R == 7 || K210_RxTemp_R == 8)) ||
			(nodesr.nowNode.nodenum == P6 && (K210_RxTemp_R == 7 || K210_RxTemp_R == 8)))
		{
			Clue_Num = K210_RxTemp_R;
			K210_Rece = 1;
			flag = 0;
			close_K210();
		}
	}

	HAL_UART_Receive_IT(&huart5, &K210_RxTemp_R, 1);
}

/*左K210接收中断*/
void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart2);

	static uint8_t flag = 0;
	if (K210_RxTemp_L == 0xff && flag == 0)
		flag = 1;
	else if (K210_RxTemp_L == 0xff && flag == 1)
		flag = 2;
	else if (flag == 2 && K210_RxTemp_L != 0xff && K210_RxTemp_L != 0x0a)
	{
		Clue_Num = K210_RxTemp_L;
		if ((nodesr.nowNode.nodenum == P1 && (K210_RxTemp_L == 3 || K210_RxTemp_L == 4)) ||
			(nodesr.nowNode.nodenum == P3 && (K210_RxTemp_L == 5 || K210_RxTemp_L == 6)) ||
			(nodesr.nowNode.nodenum == P4 && (K210_RxTemp_L == 5 || K210_RxTemp_L == 6)) ||
			(nodesr.nowNode.nodenum == P5 && (K210_RxTemp_L == 7 || K210_RxTemp_L == 8)) ||
			(nodesr.nowNode.nodenum == P6 && (K210_RxTemp_L == 7 || K210_RxTemp_L == 8)))
		{
			Clue_Num = K210_RxTemp_L;
			K210_Rece = 1;
			flag = 0;
			close_K210();
		}
	}

	HAL_UART_Receive_IT(&huart2, &K210_RxTemp_L, 1);
}
