#include "delay.h"
#include "encoder.h"
#include "motor.h"
#include "temporary_task.h"
#include "bsp_led.h"
#include "tim.h"

//延时函数+电机狗
TIM_HandleTypeDef TIM7_Handler;
void delay_init(void)
{
    //定时器7
    __HAL_RCC_TIM7_CLK_ENABLE();
     
    TIM7_Handler.Instance=TIM7;                          //通用定时器7
    TIM7_Handler.Init.Prescaler=216-1;                     //分频
    TIM7_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIM7_Handler.Init.Period=1500-1;                        //自动装载值
    TIM7_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	TIM7_Handler.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&TIM7_Handler);
	HAL_NVIC_SetPriority(TIM7_IRQn,9,0);    //设置中断优先级，抢占优先级3，子优先级3
    HAL_NVIC_EnableIRQ(TIM7_IRQn);          //开启ITM4中断  
    HAL_TIM_Base_Start_IT(&TIM7_Handler); //使能定时器7和定时器7中断 
}

//1.5ms  
void TIM7_IRQHandler(void)
{
	static uint8_t mouse = 0;    //小灯鼠
    if(__HAL_TIM_GET_IT_SOURCE(&TIM7_Handler,TIM_IT_UPDATE)==SET)//溢出中断
    {		
		mouse++;
		if(mouse>100)
		{
			mouse = 0;
			LED_C0_Toggle();
		}
	}
    __HAL_TIM_CLEAR_IT(&TIM7_Handler, TIM_IT_UPDATE);//清除中断标志位
	HAL_TIM_IRQHandler(&TIM7_Handler);
}

//最多延时1500us
void delay_us(uint16_t nus)
{
   TIM7->CNT = 0;
	 while(TIM7->CNT<nus);
}

void delay_ms(uint16_t nms)
{
	for(uint16_t i=0;i<nms;i++){
		delay_us(1000);
	}
}
