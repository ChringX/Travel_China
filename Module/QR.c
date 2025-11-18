/*
 * @File: QR.c
 * @Description: 
 * @Version: 1.0.0
 * @Author: 
 * @Date: 2023-09-13 20:33:36
 * @LastEditTime: 2023-09-15 15:39:49
 */
#include "QR.h"
#include "map.h"
#include "usart.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
int BW_num[3]={0,0,0};
uint8_t Clue_Stage[3] = {0,0,0};
#define QRBUFFER_SIZE 7
uint8_t QR_rx_buf[QRBUFFER_SIZE] = {0};
uint8_t QR_rx_len = 0;

/**
 * @brief: 
 * @return {*}
 */
void QR_receive_init(void)
{
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	HAL_UART_Receive_IT(&huart2,QR_rx_buf,QRBUFFER_SIZE);
}

/**
 * @brief: 串口2中断，做了什么事情？
 * @return {*}
 */
void USART2_IRQHandler(void)
{
	uint32_t flag_idle = 0;
	flag_idle = __HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE); 
	if((flag_idle != RESET))
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);
		HAL_UART_DMAStop(&huart2); 
		uint32_t temp = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx); 		
		QR_rx_len = QRBUFFER_SIZE - temp; 
		if(QR_rx_buf[0] == 0x48&QR_rx_buf[1]==0X45&QR_rx_buf[2]==0X41&QR_rx_buf[3]==0X44)
		{
			Clue_Stage[0] = Sixteen2Tenth(QR_rx_buf[4]);
			Clue_Stage[1] = Sixteen2Tenth(QR_rx_buf[5]);
			Clue_Stage[2] = Sixteen2Tenth(QR_rx_buf[6]);
		}
		memset(QR_rx_buf,0,QR_rx_len);
		QR_rx_len = 0;
	}
	HAL_UART_Receive_DMA(&huart2,QR_rx_buf,QRBUFFER_SIZE);
	HAL_UART_IRQHandler(&huart2);
}
//二维码1-31 2-32  以此类推

uint8_t Sixteen2Tenth(uint8_t code)
{
	uint8_t Return_Num = 0;
	switch(code)
	{
		case 0x30:
			Return_Num=0;
		    break;
		case 0x33:
			Return_Num=3;
		    break;
		case 0x34:
			Return_Num=4;
			break;
		case 0x35:
			Return_Num=5;
			break;
		case 0x36:
			Return_Num=6;
			break;
		case 0x37:
			Return_Num=7;
			break;
		case 0x38:
			Return_Num=8;
			break;
		default:
			break;
	}
	return Return_Num;
}

