#ifndef __BARRIER_H
#define __BARRIER_H

#include "sys.h"

#define Up_pitch   basic_p+12   //while(imu.pitch<Up_pitch)  出循环 刚上桥
#define Down_pitch basic_p-12   //while(imu.pitch>Down_pitch)出循环 刚下桥 
#define After_down basic_p-5    //while(imu.pitch<After_down)出循环下完 在平地
#define After_up   basic_p+5    //while(imu.pitch>After_up)出循环上完 在平地

#define Old_M_Speed    6                //老爷爷
#define QQB_Out_Speed  8                //出跷跷板
#define BL_Speed 	   12               //波浪板
#define Rubbish_Speed  13               //Rubbish
#define Stop_T_Speed   15               //原地转
#define UnderMou_Speed 20
#define GoStage_Speed  12   //16
#define Low_Speed      20 
#define Gyro_Speed     25   
#define Award_Speed    25
#define UpStage_Speed  20
#define Bridge_Speed   45   //38
#define Mount_Speed	   22
#define DownBHM_Speed  32
#define Mid_Speed      35
#define High_Speed     45
#define Champion_Speed 64

#define Green 1
#define Yellow 2
#define Red 3

extern uint8_t treasure[3];
extern uint8_t DownLiuShui;
extern uint8_t isStage;
extern uint8_t special_arrive;
extern uint8_t color_flag[5];
extern float LiuShuiRate;
extern uint8_t WavePlateLeft_Flag;
extern uint8_t WavePlateRight_Flag;

void Stage(void);
void Barrier_Bridge(void);
void Barrier_Hill(void) ;
void back(void);
void view1(void);//打景点	
void Sword_Mountain(void);
void Barrier_HighMountain(float speed);
void Barrier_Down_HighMountain(float speed);
void view(void);
void Barrier_WavedPlate(float lenght);
void South_Pole(void);
void QQB_1(void);
void door(void);
void Stage_P2(void);
void ignore_node(void);
void undermou(void);
void Special_Node(void);
void get_newroute(void);
uint8_t WaitForK210(void);
void zhunbei(void);
void Protect(float angle1);
void Connect(uint8_t Route[]);
void CGChange(float Speed);
void Motor_Control(uint8_t target_mode, float LSPEED, float RSPEED, float aim);
void Want2Go(float Dis);

#endif
