#ifndef __DELAY_H
#define __DELAY_H
#include "main.h"

extern void delay_init(void);
extern void delay_us(uint16_t nus);  //微秒级延时
extern void delay_ms(uint16_t nms);  //毫秒级延时

#endif   
