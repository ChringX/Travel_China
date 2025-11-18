#ifndef __QR_h__
#define __QR_h__
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "sys.h"

extern uint8_t Clue_Stage[3];

void QR_receive_init(void);
uint8_t Sixteen2Tenth(uint8_t code);
#endif
