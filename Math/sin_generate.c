/*
 * @File sin_generate.c
 * @Description: 
 * @Version: 1.0.0
 * @Author: 
 * @Date: 2023-09-13 20:33:34
 * @LastEditTime: 2023-09-15 15:43:07
 */
#include "sin_generate.h"
#include "math.h"

#define PI 3.1415926535f

struct sin_param sin1={0,0,50,0.1};

/**
 * @brief: 
 * @param {sin_param} *param
 * @return {*}
 */
float sin_generator(struct sin_param *param)
{
	float output;
	
	param->actual_t = param->time * param->angular_velocity;
	
	output = param->gain * sin(param->actual_t * PI/180.0f);
	
	++param->time;
	
	if (param->actual_t >= 360)
		param->time = 0;
	
	return output;
}
