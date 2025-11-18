#include "QR.h"
#include "map.h"
#include "usart.h"
#include "math.h"
#include "stdio.h"
#include "string.h"



uint8_t BW_add = 0;
uint8_t Clue_Stage[3] = {0, 0, 0};
#define QRBUFFER_SIZE 7
uint8_t QR_rx_buf[QRBUFFER_SIZE] = {0};
uint8_t QR_rx_len = 0;



/*二维码接收中断开启*/
void QR_receive_init(void)
{
	// __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	// HAL_UART_Receive_IT(&huart2,QR_rx_buf,QRBUFFER_SIZE);
}

/*二维码接收中断*/
// void USART2_IRQHandler(void)
// {
// 	// uint32_t flag_idle = 0;
// 	// flag_idle = __HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE); 
// 	// if((flag_idle != RESET))
// 	// { 
// 	// 	__HAL_UART_CLEAR_IDLEFLAG(&huart2);
// 	// 	HAL_UART_DMAStop(&huart2); 
// 	// 	uint32_t temp = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx); 		
// 	// 	QR_rx_len = QRBUFFER_SIZE - temp; 
// 	// 	if(QR_rx_buf[0] == 0x48&QR_rx_buf[1]==0X45&QR_rx_buf[2]==0X41&QR_rx_buf[3]==0X44)
// 	// 	{
// 	// 		/*ASCII码对应转换*/
// 	// 		Clue_Stage[0] = QR_rx_buf[4]-'0';
// 	// 		Clue_Stage[1] = QR_rx_buf[5]-'0';
// 	// 		Clue_Stage[2] = QR_rx_buf[6]-'0';
// 	// 	}
// 	// 	memset(QR_rx_buf,0,QR_rx_len);
// 	// 	QR_rx_len = 0;
// 	// }
// 	// HAL_UART_Receive_DMA(&huart2,QR_rx_buf,QRBUFFER_SIZE);
// 	// HAL_UART_IRQHandler(&huart2);

// 	uint32_t flag_idle = 0;
// 	flag_idle = __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE);
// 	if ((flag_idle != RESET))
// 	{
// 		__HAL_UART_CLEAR_IDLEFLAG(&huart2);
// 		HAL_UART_DMAStop(&huart2);
// 		uint32_t temp = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
// 		QR_rx_len = QRBUFFER_SIZE - temp;
// 		if (QR_rx_buf[0] == 0x48 & QR_rx_buf[1] == 0X45 & QR_rx_buf[2] == 0X41 & QR_rx_buf[3] == 0X44)
// 		{
// 			switch (QR_rx_buf[4])
// 			{ 	
// 			case 0x33:
// 				BW_add = 3;
// 				break;
// 			case 0x34:
// 				BW_add = 4;
// 				break;
// 			case 0x35:
// 				BW_add = 5;
// 				break;
// 			case 0x36:
// 				BW_add = 6;
// 				break;
// 			case 0x37:
// 				BW_add = 7;
// 				break;
// 			case 0x38:
// 				BW_add = 8;
// 				break;
// 			default:
// 				break;
// 			}
// 		}
// 		memset(QR_rx_buf, 0, QR_rx_len);
// 		QR_rx_len = 0;
// 	}
// 	HAL_UART_Receive_DMA(&huart2, QR_rx_buf, QRBUFFER_SIZE);
// 	HAL_UART_IRQHandler(&huart2);
// }
