/*
 * @File: task_create.c
 * @Description: 
 * @Version: 1.0.0
 * @Author: 
 * @Date: 2023-09-13 20:33:36
 * @LastEditTime: 2023-09-15 15:50:05
 */
#include "task_create.h"
TaskHandle_t Start_handler; // 定义开始任务句柄
TaskHandle_t main_handler;	// 定义主控任务句柄


/**
 * @brief: 开始任务创建函数
 * @return {*}
 */
void Start_task_create(void)
{
	xTaskCreate((TaskFunction_t)Start_task,		  // 任务函数
				(const char *)"Start_task",		  // 任务名字
				(uint32_t)Start_size,			  // 任务堆栈大小
				(void *)NULL,					  // 传递给任务参数的指针参数
				(UBaseType_t)Start_task_priority, // 任务的优先级
				(TaskHandle_t *)&Start_handler);  // 任务句柄
}

/**
 * @brief: 主控任务创建
 * @return {*}
 */
void main_task_create(void)
{
	xTaskCreate((TaskFunction_t)main_task,		 // 任务函数
				(const char *)"main_task",		 // 任务名字
				(uint32_t)main_size,			 // 任务堆栈大小
				(void *)NULL,					 // 传递给任务参数的指针参数
				(UBaseType_t)main_task_priority, // 任务的优先级
				(TaskHandle_t *)&main_handler);	 // 任务句柄
}
