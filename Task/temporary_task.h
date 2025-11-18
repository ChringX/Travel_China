#ifndef __temporary_task_h__
#define __temporary_task_h__
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
void GET_free_RAM(TaskHandle_t xTask);
void user_init(void);


#endif
