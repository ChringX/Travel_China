#ifndef __motor_h__
#define __motor_h__
#include "main.h"
#include "pid.h"
#include "tim.h"

/****************************************
速度：
PWM（开环）0-->编码器（闭环）
10000 --->200

****************************************/
void motor_init(void);
void motor_set_pwm(uint8_t motor, int32_t pid_out);
void pid_init(void);
void motor_pid_clear(void);

#endif
