#ifndef __openmv_h__
#define __openmv_h__
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
extern uint8_t Color_Right;
extern uint8_t Color_Left;
void Open_MV_R(void);
void Close_MV_R(void);
void Open_MV_L(void);
void Close_MV_L(void);
void MV_Init(uint32_t bound);
void send_play_command(void);
void send_stop_play_command(void);
void send_play_specified_command(uint8_t index);
extern UART_HandleTypeDef mv_R; // UART¾ä±ú
#endif
