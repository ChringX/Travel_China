#ifndef __BARRIER_H
#define __BARRIER_H

#include "sys.h"

#define basic_p   -1.35  //4.8 //5.36 //-1.4
#define Up_pitch   basic_p+12   //while(imu.pitch<Up_pitch)  出循环 刚上桥
#define Down_pitch basic_p-12   //while(imu.pitch>Down_pitch)出循环 刚下桥 
#define After_down basic_p-3    //while(imu.pitch<After_down)出循环下完 在平地
#define After_up   basic_p+3    //while(imu.pitch>After_up)出循环上完 在平地

#define Old_M_Speed    6
#define QQB_Out_Speed  8
#define BL_Speed 	   10
#define Rubbish_Speed  13
#define Stop_T_Speed   15
#define GoStage_Speed  16
#define Low_Speed      20 
#define Gyro_Speed     23//25
#define Award_Speed    25
#define Bridge_Speed   35
#define Mount_Speed	   30
#define DownBHM_Speed  32
#define Mid_Speed      35
#define High_Speed     45
#define Champion_Speed 60

extern uint8_t treasure;
extern uint8_t R_Fight;
extern uint8_t L_Fight;
extern uint8_t DownLiuShui;
extern uint8_t stage1;
extern uint8_t special_arrive;
extern uint8_t color_flag[5];
void Stage(void);
void Barrier_Bridge(float step,float speed);
void Barrier_Hill(uint8_t order) ;
void back(void);
void view1(void);//打景点	
void Sword_Mountain(void);
void Barrier_HighMountain(float speed);
void Barrier_Down_HighMountain(float speed);
void view(void);
void Barrier_WavedPlate(float lenght);
void South_Pole(float L);
void QQB_1(void);
void door(void);
void route_reset(u8 flag);
void Stage_P2(void);
void S_curve(void);
void special_node(void);
void ignore_node(void);
void undermou(void);
void Special_Node(void);
void get_newroute(void);
void Connect_Route(void);
void WaitForK210(void);
void zhunbei(void);
void Protect(float angle1);
void Connect(uint8_t Route[]);
void ConnectFirstBack(void);
void BHillChange(void);
#endif
