#ifndef __task_create_h__
#define __task_create_h__
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

//开始任务
extern TaskHandle_t Start_handler ; //定义开始任务句柄
void Start_task(void *pvParameters);//声明任务函数
#define Start_size  1024   //任务堆栈大小
#define Start_task_priority 32  //任务优先级


//主控任务
extern TaskHandle_t main_handler ; //定义主控任务句柄
void main_task(void *pvParameters);//声明任务函数
#define main_size  1024*5  //任务堆栈大小
#define main_task_priority  12//7  //12 //任务优先级



void Start_task_create(void);
void main_task_create(void);

#endif
