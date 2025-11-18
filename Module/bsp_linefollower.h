#ifndef __BSP_LINEFOLLOWER_H
#define __BSP_LINEFOLLOWER_H

#include "sys.h"

//向前的红外
#define Infrared_ahead   (uint8_t)!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_8)
//向后的红外
#define Infrared_back   (uint8_t)!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5)

extern uint8_t	infrare_open;
void get_Infrared(void);

struct Infrared_Sensor    //红外传感器
{
	uint8_t head_right;         	 //车头右边
	uint8_t head_left;         		 //车头左边
	uint8_t tail_left;          	 //车尾左边
	uint8_t tail_right;              //车尾右边
	uint8_t inside_outside_left;     //车底左边 
	uint8_t inside_outside_right;    //车底右边
};


extern volatile struct Infrared_Sensor infrared;

#endif

