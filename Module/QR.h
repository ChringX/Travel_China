#ifndef __QR_h__
#define __QR_h__
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "sys.h"

extern uint8_t Clue_Stage[3];
extern uint8_t BW_add;

void QR_receive_init(void);

#endif
