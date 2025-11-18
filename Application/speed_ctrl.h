#ifndef __SPEED_CTRL_H
#define __SPEED_CTRL_H

#include "sys.h"
#include "stdbool.h"

struct Gradual
{
	float Last;			//改变时的上一次的值
	float Now;			//当前值
	float D_value;		//改变时刻的目标值与当前值的差
};

struct Motors
{
	float Lspeed,Rspeed;		//速度
	float Cspeed;				//寻迹速度
	float Gspeed;				//自平衡速度
	float Pspeed;           	//神龙摆尾速度
	float GyroT_speedMax;		//转弯最大速度
	float GyroG_speedMax;		//自平衡最大速度
	float GyroP_speedMax;   	//漂移的最大速度
	
	float encoder_avg;			//编码器读数 
	float Distance;				//路程
	
	float Cincrement;			//循迹加速度
	float CDOWNincrement;  		//循迹减速
	float Gincrement;			//非循迹加速度  
	float GDOWNincrement;	
	
	bool is_UP;
	bool is_DOWM;
};


#define SPEED0	25
#define SPEED1	36
#define SPEED2	45
#define SPEED25	55
#define SPEED3	60
#define SPEED4	70
#define SPEED5  75

extern volatile struct Motors motor_all;
extern struct Gradual TC_speed, TG_speed,TP_speed,TCO_speed;

void gradual_cal(struct Gradual *gradual, float target, float increment1, float increment2);
void CarBrake(void);
void CarBrake_Stop(void);

#endif
