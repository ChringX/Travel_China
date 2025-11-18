#ifndef __Encoder_h__
#define __Encoder_h__
#include "main.h"
#include "tim.h"
#define FORWARD  1
#define BACKWARD -1

//定义小车轮子对应的输入捕获定时器
typedef struct{
	TIM_HandleTypeDef TIM;
	GPIO_TypeDef* GPIO_2;//B相
  uint32_t GPIO_PORT_2;
  uint32_t channelA;	
  __IO uint32_t CCR_CHAN;
}wheel;
extern wheel wheel_1,wheel_2,wheel_3,wheel_4;
/*
wheel_1     wheel_2
wheel_3     wheel_4
*/
#define MAX_pulse   3000
extern int  direction[4];                //正反转识别
extern float Real_Speed[4];

void Encoder_init(void); 
void Encoder_task(void *pvParameters);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void encoder_clear(void);

extern short Speed[4];
extern uint8_t dog[4];   //电机狗

#endif
