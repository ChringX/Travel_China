#ifndef __K210_h__
#define __K210_h__
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "sys.h"
#include "usart.h"

extern uint8_t K210_Rece;
extern uint8_t Clue_Num[2];
void K210_Enable(void);
void open_K210(void);
void close_K210(void);
#endif
