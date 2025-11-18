#include "K210.h"
#include "stdio.h"
uint8_t Clue_Num[2] = {0,0};
uint8_t K210_Rece = 0;
static uint8_t temp = 0;
static uint8_t flag = 0;
static uint8_t i = 0;

void K210_Enable(void)
{
	__HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
}

/**
 * @brief:
 * @return {*}
 */
void open_K210(void)
{
	uint8_t data = 0x55;
	HAL_UART_Transmit(&huart5, &data, 1, 0xffff);
}

/**
 * @brief:
 * @return {*}
 */
void close_K210(void)
{
	uint8_t data = 0x66;
	HAL_UART_Transmit(&huart5, &data, 1, 0xffff);
}

void UART5_IRQHandler(void)
{
	if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_RXNE) != 0)
	{
		temp = huart5.Instance->RDR;
		printf("%d\r\n",temp);
		if (flag == 2 && temp != 0xff && temp != 0x0a)
		{
			Clue_Num[i] = temp;
			i++;
			flag = 0;
		}
		if (temp == 0xff & flag == 1)
		{
			flag = 2;
		}
		if (temp == 0xff & flag == 0)
		{
			flag = 1;
		}
	}
	huart5.Instance->ISR = 0; // 清除SR标志位
	HAL_UART_IRQHandler(&huart5);
}

