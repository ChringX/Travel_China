#ifndef __SCANER_H
#define __SCANER_H
#include "sys.h"
#include "pid.h"
#define Lamp_Max 16   //循迹灯最大数
#define Lamp_Half 8
typedef struct scaner	
{
	uint16_t detail;  //二进制灯数据
	float error;			//误差
	u8 ledNum;				//灯的数量
	u8 lineNum;      //linenum用来记录有多少条引导线
}SCANER;

struct Scaner_Set {
	float CatchsensorNum;   //目标位置
	int8_t EdgeIgnore;			//忽略灯
};

extern volatile SCANER Cross_Scaner;
extern volatile struct Scaner_Set scaner_set;
extern volatile uint8_t LEFT_RIGHT_LINE;
extern float Fspeed;				//经过PID运算后的结果
extern volatile SCANER Scaner;
extern float line_weight[16];
void Go_Line(float speed);
void get_detail(void);
void Cross_getline(void);
uint8_t Line_Scan(volatile SCANER *scaner, unsigned char sensorNum, int8_t edge_ignore);
void actions(uint8_t action);
uint8_t getline_error(void);
void MODE_Switch(int8_t MODE_need);

#endif
