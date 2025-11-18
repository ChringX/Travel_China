/*
 * @File: map.c
 * @Description: 
 * @Version: 1.0.0
 * @Author: 
 * @Date: 2023-09-13 20:33:33
 * @LastEditTime: 2023-09-15 15:26:15
 */
#include "map.h"
#include "barrier.h"
#include "sys.h"
#include "usart.h"
#include "QR.h"
#include "delay.h"
#include "scaner.h"
#include "imu_task.h"
#include "turn.h"
#include "speed_ctrl.h"
#include "motor_task.h"
#include "bsp_linefollower.h"
#include "math.h"
#include "bsp_buzzer.h"
#include "motor_task.h"
#include "encoder.h"
#include "uart.h"
#include "openmv.h"
#include "motor.h"
#include "scaner.h"
struct Map_State map = {0,0};
uint8_t Turn_Flag = 0;
uint8_t Change_Head = 0;
uint8_t Change_Route = 0;
uint8_t mul2sing = 0, sing2mul = 0;
uint8_t award_flag = 0;

u8 door1route[100]={P6,N13,N12,N16,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,0XFF};
//线索为5 7   4为绿灯  
u8 door2route[100]={P6,N13,N12,N16,N18,B5,N19,C6,B7,N22,B6,N20,P7,N20,B6,N22,C9,G1,P8,0XFF};
//线索为5 8   4为绿灯
u8 door3route[100]={N12,P6,N13,N16,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,0XFF};
//线索为5 7   3为绿灯或者2是绿灯    2是黄灯
u8 door4route[100]={N12,P6,N13,N12,N16,N18,B5,N19,C6,B7,N22,B6,N20,P7,N20,B6,N22,C9,G1,P8,0XFF};
//线索为5 8   3为绿灯或者2是绿灯    2是黄灯
	
u8 door5route[100]={N9,B9,N7,P5,N7,B8,N9,C3,N14,C7,C8,C4,N20,B6,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,0XFF};
//线索为6 7   1为绿灯
u8 door6route[100]={N9,B9,N7,P5,N7,B8,N9,C3,N14,S3,N14,C7,C8,C4,N20,P7,N20,B6,N22,C9,G1,P8,0XFF};//没打S5,回去打？
//线索为6 8   1为绿灯
u8 door7route[100]={N10,N9,B9,N7,P5,N7,B8,N9,C3,N14,C7,C8,C4,N20,B6,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,0XFF};
//线索为6 7   2为绿灯或者3是绿灯    3是黄灯
u8 door8route[100]={N10,N9,B9,N7,P5,N7,B8,N9,C3,N14,C7,C8,C4,N20,P7,N20,B6,N22,C9,G1,P8,0XFF};
//线索为6 8   1为红灯 , 2或者3是绿灯    3是黄灯
	
u8 door9route[100]={N4,N3,N8,0XFF};
//线索是5 7或者5 8，4 3均为红灯或者4为红灯3为黄灯时打2
u8 door10route[100]={N4,N5,N8,0XFF};
//线索是6 7或者6 8，1 2均为红灯或者1为红灯2为黄灯时打3

u8 door11route[100] = {N10,N11,N12,P6,N13,N12,N16,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,0XFF};
//线索是5 7   4 3为红黄灯且2为红灯，1为绿灯
u8 door12route[100] = {N10,N11,N12,P6,N13,N12,N16,N18,B5,N19,C6,B7,N22,B6,N20,P7,N20,B6,N22,C9,G1,P8,0XFF};
//线索是5 8   4 3为红黄灯且2为红灯，1为绿灯
//u8 door12route[100] = {N10,N15,S4,N15,C5,N18,N16,S5,N16,N12,P6,N13,N12,N16,N18,B5,N19,C6,B7,N22,B6,N20,P7,N20,B6,N22,C9,G1,P8,0XFF};
//线索是5 8   4 3为红黄灯且2为红灯，1为绿灯
//u8 door13route[100] = {N12,N16,S5,N16,N18,C5,N15,S4,N15,N10,N9,B9,N7,P5,N7,B8,N9,C3,N14,C7,C8,C4,N20,B6,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,0XFF};
//线索是6 7   1 2为红黄灯且3为红灯，4为绿灯
u8 door13route[100] = {N12,N11,N10,N9,B9,N7,P5,N7,B8,N9,C3,N14,C7,C8,C4,N20,B6,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,0XFF};
//线索是6 7   1 2为红黄灯且3为红灯，4为绿灯
u8 door14route[100] = {N12,N11,N10,N9,B9,N7,P5,N7,B8,N9,C3,N14,C7,C8,C4,N20,P7,N20,B6,N22,C9,G1,P8,0XFF};
//线索是6 8   1 2为红黄灯且3为红灯，4为绿灯

u8 Connect5Route[100] = {N5,N12,0XFF};
//二维码为5
u8 Connect6Route[100] = {N3,N10,0XFF};
//二维码为6
			
u8 Clue1route[100]={N20,C4,C8,C7,N14,S3,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N11,N12,N5,N6,P4,N6,S2,N6,N5,N4,B2,N1,P1,N1,B1,P2,0XFF};	//不去S1、P3、p6
//线索为5 7，宝藏在P1，4是绿灯
u8 Clue2route[100]={N20,C4,C8,C7,N14,S3,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N11,N12,N5,N4,N3,P3,N3,S1,N3,N4,B3,N2,P2,0XFF};//没有P6\S2\P4
//线索为5 7，宝藏在P3，4是绿灯
u8 Clue3route[100]={N20,C4,C8,C7,N14,S3,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N11,N12,N5,N6,P4,N6,S2,N6,N5,N4,B3,N2,P2,0XFF};	//没有P6\S1\P3
//线索为5 7，宝藏在P4，4是绿灯
u8 Clue4route[100]={N20,C4,C8,C7,N14,S3,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N11,N12,N5,N6,P4,N6,S2,N6,N5,N4,N3,P3,N3,S1,N3,N4,B3,N2,P2,0XFF};	//没有P6
//线索为5 7，宝藏在P5，4是绿灯
u8 Clue5route[100]={N20,C4,C8,C7,N14,S3,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N11,N12,P6,N13,N12,N5,N6,P4,N6,S2,N6,N5,N4,B3,N2,P2,0XFF};//不去P3,S1
//线索为5 7，宝藏在P6，4是绿灯
	
u8 Clue6route[100]={N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N5,N4,B2,N1,P1,N1,B1,P2,0XFF};	//没有S1、P3
//线索为5 7，宝藏在P1，3是绿灯
u8 Clue7route[100]={N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N5,N6,P4,N6,S2,N6,N5,N4,N3,P3,N3,N4,B3,N2,P2,0XFF};	//没有P6\S1
//线索为5 7，宝藏在P3，3是绿灯
u8 Clue8route[100]={N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N5,N6,P4,N6,S2,N6,N5,N4,B3,N2,P2,0XFF};	//没有P6\S1
//线索为5 7，宝藏在P4，3是绿灯
u8 Clue9route[100]={N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N5,N6,P4,N6,S2,N6,N5,N4,B3,N2,P2,0XFF};	//没有P6\S1
//线索为5 7，宝藏在P5，3是绿灯
u8 Clue10route[100]={N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N11,N12,P6,N13,N12,N8,N5,N6,P4,N6,N5,N4,B3,N2,P2,0XFF};//没打S1	\P3
//线索为5 7，宝藏在P6，3是绿灯

u8 Clue11route[100]={N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N3,P3,N3,N4,B2,N1,P1,N1,B1,P2,0XFF};	//没有S2\P4
//线索为5 7，宝藏在P1，2是绿灯
u8 Clue12route[100]={N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N3,S1,N3,P3,N3,N4,B3,N2,P2,0XFF};	////没有P6\P4\S2
//线索为5 7，宝藏在P3，2是绿灯
u8 Clue13route[100]={N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N3,N4,N5,N6,P4,N6,N5,N4,B3,N2,P2,0XFF};	//没有P6\P3\S1
//线索为5 7，宝藏在P4，2是绿灯
u8 Clue14route[100]={N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N3,S1,N3,P3,N3,N4,N5,N6,P4,N6,S2,N6,N5,N4,B3,N2,P2,0XFF};	//没有P6
//线索为5 7，宝藏在P5，2是绿灯
u8 Clue15route[100]={N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N11,N12,P6,N13,N12,N8,N3,N4,B3,N2,P2,0XFF};	//没有S2,P4
//线索为5 7，宝藏在P6，2是绿灯

u8 Clue16route[100]={N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N3,P3,N3,N4,B2,N1,P1,N1,B1,P2,0XFF};	//不去S2\P4\S1
//线索为5 7，宝藏在P1，1是绿灯
u8 Clue17route[100]={N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N3,S1,N3,P3,N3,N4,B3,N2,P2,0XFF};	////没有P6\P4\S2
//线索为5 7，宝藏在P3，1是绿灯                                                                                                 
u8 Clue18route[100]={N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N3,N4,N5,N6,P4,N6,N5,N4,B3,N2,P2,0XFF};	//没有P6\P3\S1
//线索为5 7，宝藏在P4，1是绿灯                                                                                                 
u8 Clue19route[100]={N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N3,S1,N3,P3,N3,N4,B3,N2,P2,0XFF};	//没有P6
//线索为5 7，宝藏在P5，1是绿灯
u8 Clue20route[100]={N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N11,N12,P6,N13,N12,N11,N10,N3,N4,B3,N2,P2,0XFF};	//没有P4,S2,S3,看看要加速还是删掉P3//P3,S1S
//线索为5 7，宝藏在P6，1是绿灯
	
u8 Clue21route[100]={G1,C9,N22,B6,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N11,N12,N5,N6,P4,N6,S2,N6,N5,N4,B2,N1,P1,N1,B1,P2,0XFF};	//不去S1\P3
//线索为5 8，宝藏在P1，4是绿灯              
u8 Clue22route[100]={G1,C9,N22,B6,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N11,N12,N5,N6,P4,N6,S2,N6,N5,N4,N3,P3,N3,N4,B3,N2,P2,0XFF};//没有P6、S1
//线索为5 8，宝藏在P3，4是绿灯              
u8 Clue23route[100]={G1,C9,N22,B6,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N11,N12,N5,N6,P4,N6,S2,N6,N5,N4,N3,P3,N3,N4,B3,N2,P2,0XFF};	//没有P6\S1
//线索为5 8，宝藏在P4，4是绿灯              
u8 Clue24route[100]={G1,C9,N22,B6,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N11,N12,N5,N6,P4,N6,S2,N6,N5,N4,N3,P3,N3,N4,B3,N2,P2,0XFF};	//没有P6\S1
//线索为5 8，宝藏在P5，4是绿灯              
u8 Clue25route[100]={G1,C9,N22,B7,C6,N19,B5,N18,N16,S5,N16,N12,P6,N13,N12,N5,N6,P4,N6,S2,N6,N5,N4,N3,P3,N3,N4,B3,N2,P2,0XFF};//不去P3,S1\S4，测测！！！
//线索为5 8，宝藏在P6，4是绿灯
	
u8 Clue26route[100]={G1,C9,N22,B6,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N5,N6,P4,N6,N5,N4,B2,N1,P1,N1,B1,P2,0XFF};	//没有S1、P3\S1\s3\s2
//线索为5 8，宝藏在P1，3是绿灯              
u8 Clue27route[100]={G1,C9,N22,B6,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N5,N4,N3,P3,N3,S1,N3,N4,B3,N2,P2,0XFF};	//没有P6\P4\S2,CCCCCCCCC
//线索为5 8，宝藏在P3，3是绿灯              
u8 Clue28route[100]={G1,C9,N22,B6,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N5,N6,P4,N6,S2,N6,N5,N4,N3,P3,N3,N4,B3,N2,P2,0XFF};	//没有P6\P3\S1
//线索为5 8，宝藏在P4，3是绿灯              
u8 Clue29route[100]={G1,C9,N22,B6,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N5,N6,P4,N6,S2,N6,N5,N4,N3,P3,N3,N4,B3,N2,P2,0XFF};	//没有P6
//线索为5 8，宝藏在P5，3是绿灯              
u8 Clue30route[100]={G1,C9,N22,B7,C6,N19,B5,N18,N16,S5,N16,N12,P6,N13,N12,N8,N5,N6,P4,N6,S2,N6,N5,N4,B3,N2,P2,0XFF};//没打S1	/p4.s2.p3
//线索为5 8，宝藏在P6，3是绿灯

u8 Clue31route[100]={G1,C9,N22,B6,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N3,N4,B2,N1,P1,N1,B1,P2,0XFF};	//没有S2\P4\P3\S1???????CCCCCCCCCC
//线索为5 8，宝藏在P1，2是绿灯              
u8 Clue32route[100]={G1,C9,N22,B6,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N3,S1,N3,P3,N3,N4,B3,N2,P2,0XFF};	////没有P6\P4\S1CC?
//线索为5 8，宝藏在P3，2是绿灯              
u8 Clue33route[100]={G1,C9,N22,B6,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N3,N4,N5,N6,P4,N6,N5,N4,B3,N2,P2,0XFF};	//没有P6//p3/s1
//线索为5 8，宝藏在P4，2是绿灯              
u8 Clue34route[100]={G1,C9,N22,B6,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N3,S1,N3,P3,N3,N4,B3,N2,P2,0XFF};	//没有P6//p4/s2
//线索为5 8，宝藏在P5，2是绿灯              
u8 Clue35route[100]={G1,C9,N22,B7,C6,N19,B5,N18,N16,S5,N16,N12,P6,N13,N12,N8,N3,S1,N3,P3,N3,N4,B3,N2,P2,0XFF};	//没有S2\P4\s1\p3\s3
//线索为5 8，宝藏在P6，2是绿灯

u8 Clue36route[100]={G1,C9,N22,B6,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N3,N4,B2,N1,P1,N1,B1,P2,0XFF};	//不去S2\P4\S1
//线索为5 8，宝藏在P1，1是绿灯              
u8 Clue37route[100]={G1,C9,N22,B6,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N3,S1,N3,P3,N3,N4,B3,N2,P2,0XFF};	////没有P6\P4\S2
//线索为5 8，宝藏在P3，1是绿灯                                                                                                    
u8 Clue38route[100]={G1,C9,N22,B6,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N3,N4,N5,N6,P4,N6,N5,N4,B3,N2,P2,0XFF};	//没有P6\P3\S1
//线索为5 8，宝藏在P4，1是绿灯                                                                                                    
u8 Clue39route[100]={G1,C9,N22,B6,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N3,S1,N3,P3,N3,N4,B3,N2,P2,0XFF};	//没有P6//CCCCCCCCC10.08
//线索为5 8，宝藏在P5，1是绿灯              
u8 Clue40route[100]={G1,C9,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N16,S5,N16,N18,C5,N15,S4,N15,N10,N3,P3,N3,N4,B3,N2,P2,0XFF};	//没有P4,S2\P3\S1?????????????//P5
//线索为5 8，宝藏在P6，1是绿灯
	
u8 Clue41route[100]={N20,B6,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N5,N4,B2,N1,P1,N1,B1,P2,0XFF};	//没打S1\P3
//线索为6 7，宝藏在P1，4是绿灯
u8 Clue42route[100]={N20,B6,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N5,N6,P4,N6,N5,N4,N3,P3,N3,N4,B3,N2,P2,0XFF};	//S2(P4要吗）CCCCCCCCC
//线索为6 7，宝藏在P3，4是绿灯
u8 Clue43route[100]={N20,B6,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N5,N6,P4,N6,S2,N6,N5,N4,B3,N2,P2,0XFF};	//S1\S2
//线索为6 7，宝藏在P4，4是绿灯
u8 Clue44route[100]={N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N11,N12,N5,N4,B3,N2,P2,0XFF};	//没打S1,P3  时间够的话改成不打P6,打S1,P3,CCCCCCCC
//线索为6 7，宝藏在P5，4是绿灯
u8 Clue45route[100]={N20,B6,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N5,N6,P4,N6,S2,N6,N5,N4,B3,N2,P2,0XFF};	//S1/P3??C?
//线索为6 7，宝藏在P6，4是绿灯
	
u8 Clue46route[100]={N20,B6,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N8,N5,N4,B2,N1,P1,N1,B1,P2,0XFF};//不打S1\P3
//线索为6 7，宝藏在P1，3是绿灯
u8 Clue47route[100]={N20,B6,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N8,N5,N6,P4,N6,N5,N4,N3,P3,N3,N4,B3,N2,P2,0XFF};	//S1,CCCCC
//线索为6 7，宝藏在P3，3是绿灯
u8 Clue48route[100]={N20,B6,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N8,N5,N6,P4,N6,S2,N6,N5,N4,B3,N2,P2,0XFF};	//S1,CC
//线索为6 7，宝藏在P4，3是绿灯
u8 Clue49route[100]={N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N5,N4,B3,N2,P2,0XFF};//不打P6	\P3\S1
//线索为6 7，宝藏在P5，3是绿灯
u8 Clue50route[100]={N20,B6,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N8,N5,N6,P4,N6,S2,N6,N5,N4,B3,N2,P2,0XFF};	//P3/S1
//线索为6 7，宝藏在P6，3是绿灯

	
u8 Clue51route[100]={N20,B6,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N8,N3,P3,N3,N4,B2,N1,P1,N1,B1,N2,P2,0XFF};	//没打S2,P4\S1
//线索为6 7，宝藏在P1，2是绿灯        
u8 Clue52route[100]={N20,B6,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N8,N3,S1,N3,P3,N3,N4,B3,N2,P2,0XFF};	//P4,S2
//线索为6 7，宝藏在P3，2是绿灯        
u8 Clue53route[100]={N20,B6,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N8,N3,N4,N5,N6,P4,N6,S2,N6,N5,N4,B3,N2,P2,0XFF};	//P3,S1
//线索为6 7，宝藏在P4，2是绿灯
u8 Clue54route[100]={N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N3,P3,N3,N4,B3,N2,P2,0XFF};	//先测测再看看该不该？
//线索为6 7，宝藏在P5，2是绿灯
u8 Clue55route[100]={N20,B6,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N8,N3,S1,N3,P3,N3,N4,B3,N2,P2,0XFF};//P4,S2
//线索为6 7，宝藏在P6，2是绿灯

u8 Clue56route[100]={N20,B6,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N11,N10,N3,S1,N3,P3,N3,N4,B2,N1,P1,N1,B1,P2,0XFF};	//不打S2\P4
//线索为6 7，宝藏在P1，1是绿灯
u8 Clue57route[100]={N20,B6,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N11,N10,N3,S1,N3,P3,N3,N4,B3,N2,P2,0XFF};	//S2
//线索为6 7，宝藏在P3，1是绿灯
u8 Clue58route[100]={N20,B6,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N11,N10,N3,N4,N5,N6,P4,N6,N5,N4,B3,N2,P2,0XFF};	//S2
//线索为6 7，宝藏在P4，1是绿灯
u8 Clue59route[100]={N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N3,S1,N3,P3,N3,N4,B3,N2,P2,0XFF};	//不打P6\S2
//线索为6 7，宝藏在P5，1是绿灯
u8 Clue60route[100]={N20,B6,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N11,N10,N3,S1,N3,P3,N3,N4,B3,N2,P2,0XFF};	//P4/S2
//线索为6 7，宝藏在P6，1是绿灯	
	
	
u8 Clue61route[100]={G1,C9,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N5,N6,P4,N6,N5,N4,B2,N1,P1,N1,B1,P2,0XFF};	//不打S1\P3
//线索为6 8，宝藏在P1，4是绿灯
u8 Clue62route[100]={G1,C9,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N5,N4,N3,P3,N3,N4,B3,N2,P2,0XFF};	//P4\S2
//线索为6 8，宝藏在P3，4是绿灯
u8 Clue63route[100]={G1,C9,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N5,N6,P4,N6,S2,N6,N5,N4,B3,N2,P2,0XFF};	//P3\S1
//线索为6 8，宝藏在P4，4是绿灯
u8 Clue64route[100]={G1,C9,N22,B6,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N11,N12,N5,N4,B3,N2,P2,0XFF};	//不打S1,P3,P4,S2    时间够的话改成不打P6,打S1,P3CCCC
//线索为6 8，宝藏在P5，4是绿灯
u8 Clue65route[100]={G1,C9,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N5,N6,P4,N6,S2,N6,N5,N4,B3,N2,P2,0XFF};	//S1 //C?
//线索为6 8，宝藏在P6，4是绿灯
	
u8 Clue66route[100]={G1,C9,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N8,N5,N6,P4,N6,N5,N4,B2,N1,P1,N1,B1,P2,0XFF};//不打S1	\P3
//线索为6 8，宝藏在P1，3是绿灯    
u8 Clue67route[100]={G1,C9,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N8,N5,N4,N3,P3,N3,N4,B3,N2,P2,0XFF};	//P4/S2
//线索为6 8，宝藏在P3，3是绿灯    
u8 Clue68route[100]={G1,C9,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N8,N5,N6,P4,N6,S2,N6,N5,N4,B3,N2,P2,0XFF};	//P3/S2
//线索为6 8，宝藏在P4，3是绿灯
u8 Clue69route[100]={G1,C9,N22,B6,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N5,N4,B3,N2,P2,0XFF};//不打P6/S1/p3/P4/S2
//线索为6 8，宝藏在P5，3是绿灯
u8 Clue70route[100]={G1,C9,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N8,N5,N6,P4,N6,S2,N6,N5,N4,B3,N2,P2,0XFF};	//P3/S1
//线索为6 8，宝藏在P6，3是绿灯

u8 Clue71route[100]={G1,C9,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N8,N3,P3,N3,N4,B2,N1,P1,N1,B1,P2,0XFF};	//没打S2,P4/S1/CC?
//线索为6 8，宝藏在P1，2是绿灯
u8 Clue72route[100]={G1,C9,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N8,N3,S1,N3,P3,N3,N4,B3,N2,P2,0XFF};	//P4/S2
//线索为6 8，宝藏在P3，2是绿灯
u8 Clue73route[100]={G1,C9,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N8,N3,N4,N5,N6,P4,N6,N5,N4,B3,N2,P2,0XFF};	//P3/S1
//线索为6 8，宝藏在P4，2是绿灯
u8 Clue74route[100]={G1,C9,N22,B6,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N3,N4,B3,N2,P2,0XFF};	//不打P6//S2/S1//S1-S5
//线索为6 8，宝藏在P5，2是绿灯
u8 Clue75route[100]={G1,C9,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N8,N3,S1,N3,P3,N3,N4,B3,N2,P2,0XFF};//P4/S2
//线索为6 8，宝藏在P6，2是绿灯

u8 Clue76route[100]={G1,C9,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N11,N10,N3,S1,N3,N4,B2,N1,P1,N1,B1,P2,0XFF};	//不打S2/P4	
//线索为6 8，宝藏在P1，1是绿灯    
u8 Clue77route[100]={G1,C9,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N11,N10,N3,S1,N3,P3,N3,N4,B3,N2,P2,0XFF};	//S2
//线索为6 8，宝藏在P3，1是绿灯    
u8 Clue78route[100]={G1,C9,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N11,N10,N3,N4,N5,N6,P4,N6,S2,N6,N5,N4,B3,N2,P2,0XFF};	//S2
//线索为6 8，宝藏在P4，1是绿灯
u8 Clue79route[100]={G1,C9,N22,B6,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N3,N4,B3,N2,P2,0XFF};	//不跑P6/S2
//线索为6 8，宝藏在P5，1是绿灯
u8 Clue80route[100]={G1,C9,N22,B7,C6,N19,B5,N18,N16,N12,P6,N13,N12,N11,N10,N3,S1,N3,P3,N3,N4,B3,N2,P2,0XFF};	//P4\S2
//线索为6 8，宝藏在P6，1是绿灯
	
NODESR nodesr;	//运作中间变量
	
/***************出发***************/
u8 route[100] = {B1,N1,P1,N1,B2,N4,0XFF};

/***************长桥***************/
//u8 route[100] = {B1,N1,0XFF};

/***************楼梯***************/
//u8 route[100] = {N19,C6,B7,N22,0XFF};

/***************波浪板+南极***************/
//u8 route[100] = {G1,P8,G1,C9,0XFF};

/***************珠峰***************/
//u8 route[100] = {P7,N20,0XFF};

/***************刀山***************/
//u8 route[100] = {B6,N20,0XFF};

/***************跷跷板***************/
//u8 route[100] = {B9,N7,P5,N7,B8,N9,N10,0XFF};

/***************测试时间***************/
//u8 route[100] = {N12,N16,S5,N16,N18,0XFF};
/**
 * @brief: 地图初始化
 * @return {*}
 * @note {B1,RIGHT_LINE,0,30,5000,BBridge}
 */
void mapInit()
{
	u8 i=0;
	nodesr.flag=0;
	
/***************选择头***************/
	ScanMode = is_Front;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
	
//	ScanMode = is_Back;
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
	
/***************出发***************/
	nodesr.nowNode.nodenum = N2;		
	nodesr.nowNode.angle = 0;		
	nodesr.nowNode.function = 1;	
	nodesr.nowNode.speed = 25;          
	nodesr.nowNode.step= 10*60;            
	nodesr.nowNode.flag = CLEFT|RIGHT_LINE; 

/***************长桥***************/	
//	nodesr.nowNode.nodenum = N2;		
//	nodesr.nowNode.angle = 0;		
//	nodesr.nowNode.function = 1;	
//	nodesr.nowNode.speed = 15;           
//	nodesr.nowNode.step= 10*60;            
//	nodesr.nowNode.flag = CLEFT;    

/***************楼梯***************/
//	nodesr.nowNode.nodenum = B5;		
//	nodesr.nowNode.angle = 180;		
//	nodesr.nowNode.function = BHill;	
//	nodesr.nowNode.speed = 15;          
//	nodesr.nowNode.step= 10*60;            
//	nodesr.nowNode.flag = 1;   

/***************波浪板+南极***************/
//	nodesr.nowNode.nodenum = G1;		
//	nodesr.nowNode.angle = 180;		
//	nodesr.nowNode.function = BLBL;	
//	nodesr.nowNode.speed = 15;//300             
//	nodesr.nowNode.step= 10*60;//60               
//	nodesr.nowNode.flag = 1;    
	
/***************珠峰***************/
//	nodesr.nowNode.nodenum = N20;		
//	nodesr.nowNode.angle = 0;		
//	nodesr.nowNode.function = 1;	
//	nodesr.nowNode.speed = 15;          
//	nodesr.nowNode.step= 10*60;            
//	nodesr.nowNode.flag = CRIGHT;   

/***************刀山***************/
//	nodesr.nowNode.nodenum = N22;		
//	nodesr.nowNode.angle = 0;		
//	nodesr.nowNode.function = 1;	
//	nodesr.nowNode.speed = 15;        
//	nodesr.nowNode.step= 10 * 60.0;         
//	nodesr.nowNode.flag = DLEFT;   

/***************跷跷板***************/
//	nodesr.nowNode.nodenum = N9;		
//	nodesr.nowNode.angle = 180;		
//	nodesr.nowNode.function = 1;	
//	nodesr.nowNode.speed = 15;           
//	nodesr.nowNode.step= 10 * 60.0;          
//	nodesr.nowNode.flag = MUL2MUL|RIGHT_LINE;   

/***************测试时间***************/
//nodesr.nowNode.nodenum = N8;		
//nodesr.nowNode.angle = 35;		
//nodesr.nowNode.function = 1;	
//nodesr.nowNode.speed = 23;           
//nodesr.nowNode.step= 10 * 60.0;          
//nodesr.nowNode.flag = DRIGHT|DLEFT;

	//全地图调整
	for(i=0;i<126;i++)			
	{
		Node[i].step*=63.3f;
	}
}

/**
 * @brief: 第二次出发初始化
 * @return {*}
 */
void mapInit1(void)
{
	nodesr.nowNode.nodenum = N2;		//起始点   //N2
	nodesr.nowNode.angle = 0;		//起始角度   //0
	nodesr.nowNode.function = 1;	//起始函数   //1
	nodesr.nowNode.speed = 25;//300             
	nodesr.nowNode.step= 10*60;//60               
	nodesr.nowNode.flag = CLEFT|RIGHT_LINE;    //CLEFT|RIGHT_LINE
    nodesr.flag=0;
}
	
/**
 * @brief: 
 * @param {u8} nownode
 * @param {u8} nextnode
 * @return {*}
 */
u8 getNextConnectNode(u8 nownode,u8 nextnode) 
{//得到本节点到相邻结点的地址
	unsigned char rest = ConnectionNum[nownode];//这个结点相邻的结点数
	unsigned char addr = Address[nownode];//得到结点的addr
	int i = 0;
	for (i = 0; i < rest; i++) 
	{
		if(Node[addr].nodenum == nextnode)//返回结点地址
		{			
			return addr;
		}
		addr++;
	}
	return 0;
}

/**
 * @brief: 
 * @return {*}
 */
u8 deal_arrive()
{				
	register uint8_t lnum = 0, i = 0;
	register uint16_t seed = 0;
	if ((nodesr.nowNode.flag & DLEFT) == DLEFT)  //左半边
	{
		//左边6个灯任意5个亮即可
		if (Cross_Scaner.ledNum>=5)
		{
			seed = 0X8000;
			for (i = 0; i<6; i++)
			{
				if (Cross_Scaner.detail & seed)
					++lnum;
				if (lnum >= 5)
				{
					return 1;
				}
				seed >>= 1;
			}
			lnum = 0;
		}
	}
	if ((nodesr.nowNode.flag & DRIGHT) == DRIGHT)//右半边
	{    	
		if (Cross_Scaner.ledNum >= 5)
		{
			seed = 0X0001;
			for (i = 0; i<6; i++)
			{
				if (Cross_Scaner.detail & seed)
					++lnum;
				if (lnum >= 5)
				{
					return 1;
				}
				seed <<= 1;
			}
			lnum = 0;
		}
	}

	if ((nodesr.nowNode.flag & CLEFT) == CLEFT)//左分岔路
	{
		//左边数起第二、第三个灯任意一个亮即可
		 if( (Cross_Scaner.ledNum>=4&&Cross_Scaner.ledNum<=7) && ((Cross_Scaner.detail&0x4000)|(Cross_Scaner.detail&0x2000)) )//
		 {
			return 1;
		}
	}
	if ((nodesr.nowNode.flag & MCLEFT) == MCLEFT)//左分岔路
	{
		//左边数起第一个灯亮即可
		 if( (Cross_Scaner.ledNum>=4&&Cross_Scaner.ledNum<=7) && (Cross_Scaner.detail&0x8000) )
		{
			return 1;
		}
	}
	if ((nodesr.nowNode.flag & MCRIGHT) == MCRIGHT)//左分岔路
	{
		//右边数起第一个灯亮即可
		 if( (Cross_Scaner.ledNum>=4&&Cross_Scaner.ledNum<=7) && (Cross_Scaner.detail&0x0001) )
		{
			return 1;
		}
	}
	if ((nodesr.nowNode.flag & CRIGHT) == CRIGHT)//右分岔路
	{
		 if( (Cross_Scaner.ledNum>=4&&Cross_Scaner.ledNum<=7) && (Cross_Scaner.detail&0xc) )//右起2和3灯亮
		{
			return 1;
		}
	}
	if ((nodesr.nowNode.flag & MORELED) == MORELED)
	{
		 if( (Cross_Scaner.ledNum>=5) )//5个灯以上亮
		{
			return 1;
		}
	}
	else if ((nodesr.nowNode.flag & AWHITE) == AWHITE)//全白
	{
		 if((Cross_Scaner.ledNum>=10&&(Cross_Scaner.detail&0x1FF8)==0x1FF8))
		{
			return 1;
		}
	}
	
	else if ((nodesr.nowNode.flag & MUL2SING) == MUL2SING)//三分岔路
	{
		if (Cross_Scaner.lineNum > 1 && Cross_Scaner.ledNum >= 4)
			++mul2sing;
		if (mul2sing > 4 && Cross_Scaner.lineNum == 1) //线数目由多变成一条
		{
			mul2sing = sing2mul = 0;
			return 1;
		}
	}
	
	else if ((nodesr.nowNode.flag & MUL2MUL) == MUL2MUL)  //线数目由多条变多条
	{
		if (Cross_Scaner.lineNum > 1 && Cross_Scaner.ledNum >= 4)
			++mul2sing;
		if (mul2sing > 4 && (Cross_Scaner.lineNum == 1 || Cross_Scaner.ledNum <=3 ))
			++sing2mul;
		if (sing2mul > 4 && Cross_Scaner.lineNum > 1 && Cross_Scaner.ledNum >= 4)
		{
			mul2sing = sing2mul = 0;
			return 1;
		}
	}
	return 0;
}


/**
 * @brief: 
 * @return {*}
 */
void Cross(void)
{
	float num = 0;
	static u8 _flag=1;			//结点动作标志位
	if(map.point == 0)
	{
		nodesr.nextNode	= Node[getNextConnectNode(nodesr.nowNode.nodenum,route[map.point++])];//获取下一结点
	}
	if(_flag==1)//循迹
	{
		pid_mode_switch(is_Line);
	    if(fabsf(motor_all.Distance) >= 0.7f*nodesr.nowNode.step) //距离大于70cm就不循迹了
		{
			_flag=0;
			if(((nodesr.nowNode.flag&RESTMPUZ)==RESTMPUZ))//陀螺仪校正
			{
				if ((Scaner.detail & 0X0180) == 0X0180) //如果在最中间位置
				{
					mpuZreset(imu.yaw, nodesr.nowNode.angle);     //获取补偿角Z;
				}
			}
			if( (fabs(need2turn(getAngleZ(),nodesr.nextNode.angle))<=10) || (fabs(need2turn(nodesr.nowNode.angle,nodesr.nextNode.angle))<=10))
			{								
				motor_all.Cspeed = nodesr.nowNode.speed;	//不用减速转弯，靠循迹转弯
			}
			else
			{
				if ((nodesr.nowNode.flag & SLOWDOWN) == SLOWDOWN)
				{
					motor_all.Cspeed = Gyro_Speed;
				}
				else if((nodesr.nowNode.flag & STOPTURN) != STOPTURN)//陀螺仪校正
				{
					motor_all.Cspeed = Gyro_Speed;
				}
				else
				{
					motor_all.Cspeed = Gyro_Speed/*0.7f * nodesr.nowNode.speed*/;// 原来是0.5    0.9当前最好
				}
				
			}
		}
		else//未达到0.7的距离
		{
			static int award_time = 0;
			if((nodesr.nowNode.step >= 40*63.3f) && (award_flag == 0) && (nodesr.nowNode.angle != nodesr.lastNode.angle))
			{
				if(award_time < 500)
				{
					if(Scaner.detail & 0X180)
						award_time += 5;
					else
						award_time +=1;
				}
				else
				{
					award_flag = 1;
					award_time = 0;
				}
			}
			else
			{
				award_flag = 1;
			}
			
			if(award_flag)
			{
				line_pid_param.kp = 22; // 22
				line_pid_param.ki = 0;
				line_pid_param.kd = 8; // 8.5
				motor_all.Cspeed = nodesr.nowNode.speed;
			}
			else
			{
				line_pid_param.kp = 26;
				line_pid_param.ki = 0;
				line_pid_param.kd = 5;
				motor_all.Cspeed = Award_Speed;
			}
		}
	}
	else if(_flag==0)//路径超过0.7  判断路口
	{
		if((nodesr.nowNode.flag & Temp_L) == Temp_L)
		{
			LEFT_RIGHT_LINE = 1;
		}
		else if((nodesr.nowNode.flag & Temp_R) == Temp_R)
		{
			LEFT_RIGHT_LINE = 2;
		}
		else if((nodesr.nowNode.flag & Temp_LiuShui) == Temp_LiuShui)
		{
			LEFT_RIGHT_LINE = 3;
		}
		map_function(nodesr.nowNode.function);
		if((nodesr.flag&0x04)!=0x04&&(nodesr.flag&0x80)!=0x80&&(nodesr.flag&0x20)!=0x20)	//判断是否到达路口
		{
			Cross_getline();
			while(!deal_arrive())
			{
				vTaskDelay(2);
				Cross_getline();
			}
			send_play_specified_command(35);//识别到路口
			scaner_set.CatchsensorNum = 0;
			nodesr.flag |= 0x04;	//到达路口		
		}	
		if(((nodesr.flag&0x04)!=0x04&&special_arrive==1))//特殊的切换循迹结点
		{
			scaner_set.CatchsensorNum = 0;
			nodesr.flag |= 0x04;	//到达路口
			special_arrive=0;
		}
	}
	if((nodesr.flag&0x04)==0x04)//转弯（已经到达路口）
	{
		nodesr.flag&=~0x04;		//	清除到达路口标志
		if(route[map.point-1] != 255)	 //route[i]==255代表一整条路线走完
		{
		  //如果下一结点角度与当前结点角度相同，不需要转，不需要减速	    
			if ((fabs(need2turn(getAngleZ(),nodesr.nextNode.angle))<10) 
				||(fabs(need2turn(nodesr.nowNode.angle,nodesr.nextNode.angle))<10)
				||(nodesr.nextNode.flag&NOTURN)==NOTURN
				||(nodesr.nowNode.nodenum==S1)
				||(nodesr.nowNode.nodenum==S2)
				||(route[map.point-3]==S3)
				||(route[map.point-3]==S4)
				||(route[map.point-3]==S5)
				||(stage1 == 1))
			{		
				stage1 = 0;
				_flag=1;
				if(nodesr.nowNode.nodenum == N6 && nodesr.nextNode.nodenum == N5)
				{
					num = motor_all.Distance;
					while(fabsf(motor_all.Distance - num) < 8 * 60)
						vTaskDelay(2);
				}
			}
			else //需要转
			{	 
				if(nodesr.nowNode.flag&DRIFT)//本来想用自平衡走曲线？
				{
					pid_mode_switch(is_Gyro);
					struct PID_param origin_parm=gyroG_pid_param;
					gyroG_pid_param.kp=0.95;
					float origin_speedMax=motor_all.GyroG_speedMax;
					
					num = motor_all.Distance;
					motor_all.Gspeed=motor_all.Cspeed;
					motor_all.GyroG_speedMax=0.45f*motor_all.Gspeed;
					motor_all.Gincrement=20;
					angle.AngleG=nodesr.nextNode.angle;
//					if(nodesr.nowNode.nodenum==N7)
//					{
//						GyroP_pid_param.kp=1.65;
//					}
					while(fabsf(nodesr.nextNode.angle-getAngleZ()) >2)
					{
//						Drift(motor_all.Pspeed,nodesr.nextNode.angle);
						if(Scaner.lineNum==1&&(Scaner.detail&0x7E0)&&(fabs(need2turn(angle.AngleG,getAngleZ()))<fabs(need2turn(angle.AngleG,nodesr.nowNode.angle))*0.2f))
						{
							break;
						}
						vTaskDelay(2);
					}
					gyroG_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
					gyroG_pid_param=origin_parm;
					motor_all.GyroG_speedMax=origin_speedMax;
				}
				else if(nodesr.nowNode.flag&L_follow)//左循迹加大kp的转弯，适用小角度，多线干扰陀螺仪摆尾位置不合适
				{
					float original_line_pid = line_pid_param.kp;
					float max = line_pid_param.outputMax;
					line_pid_param.kp = 70;   //增大KP辅助转弯//80
					line_pid_param.ki = 0;
					line_pid_param.kd = 5;
					line_pid_param.outputMax = 0.6f*motor_all.Cspeed;
					nodesr.nowNode.flag|=LEFT_LINE;
					angle.AngleG = nodesr.nextNode.angle;
              				
					encoder_clear();
					float num = motor_all.Distance;
					while(fabsf(motor_all.Distance - num) < 3*60)
						vTaskDelay(2);
					while(fabs(need2turn(angle.AngleG,getAngleZ()))>4) //2
					{   //角度达到要求
						vTaskDelay(2);
						getline_error();
						if(Scaner.lineNum==1&&((Scaner.detail&0x180)!=0)&&(fabsf(need2turn(angle.AngleG,getAngleZ()))<fabsf(need2turn(angle.AngleG,nodesr.nowNode.angle))*0.4f))
						{  
							break;
						}
					}
					nodesr.nowNode.flag&=(~LEFT_LINE);//取消左循迹标志位
					line_pid_param.kp = original_line_pid;  //恢复正常
					line_pid_param.outputMax =  max;
				}
				else if(nodesr.nowNode.flag&R_follow)//左循迹加大kp的转弯，适用小角度，多线干扰陀螺仪摆尾位置不合适
				{
					float original_line_pid = line_pid_param.kp;
					float max = line_pid_param.outputMax;
					float num=motor_all.Distance;
					if(nodesr.nowNode.nodenum==N4)
					{
						while(fabsf(motor_all.Distance-num) < 5*60)
						{
							vTaskDelay(2);
						}
					}
					line_pid_param.kp = 65;   //增大KP辅助转弯//80//70
					line_pid_param.ki = 0;
					line_pid_param.kd = 10;
					line_pid_param.outputMax = 0.6f*motor_all.Cspeed;
					nodesr.nowNode.flag|=RIGHT_LINE;
					angle.AngleG = nodesr.nextNode.angle;
                   				
					while(fabs(need2turn(angle.AngleG,getAngleZ()))>4)
					{   //角度达到要求
						vTaskDelay(2);
						getline_error();
						if((Scaner.lineNum==1||Scaner.lineNum==2)&&((Scaner.detail&0x7E0)!=0)&&(fabs(need2turn(angle.AngleG,getAngleZ()))<fabs(need2turn(angle.AngleG,nodesr.nowNode.angle))*0.5f))
						{  
									break;
						}
					}
					nodesr.nowNode.flag&=(~RIGHT_LINE);//取消左循迹标志位
					line_pid_param.kp = original_line_pid;  //恢复正常
					line_pid_param.outputMax =  max;
				}
				else		//差速转
				{
					//判断
					float now_angle = nodesr.nowNode.angle;
					float back_angle = Modified_Angle(back_angle,nodesr.nowNode.angle);
					
					if((nodesr.nowNode.nodenum == N20 && nodesr.nextNode.nodenum == C4 && nodesr.lastNode.nodenum == P7)
					|| (nodesr.nowNode.nodenum == N7 && nodesr.nextNode.nodenum == P5 && ScanMode == is_Back && Clue_Stage[1] == 6)
					|| (nodesr.lastNode.nodenum == C7 && nodesr.nowNode.nodenum == C8 && nodesr.nextNode.nodenum == C4 && ScanMode == is_Front && Clue_Stage[2] == 8)
					|| (nodesr.nowNode.nodenum == C9 && nodesr.nextNode.nodenum == G1 && ScanMode == is_Back && Clue_Stage[2] == 7)
					|| (nodesr.lastNode.nodenum == N5 && nodesr.nowNode.nodenum == N12 && nodesr.nextNode.nodenum == P6 && ScanMode == is_Back && Clue_Stage[1] == 5)
					|| (nodesr.nowNode.nodenum == C6 && nodesr.nextNode.nodenum == B7 && ScanMode == is_Back && Clue_Stage[2] == 6)
					|| (nodesr.nowNode.nodenum == N10 && nodesr.lastNode .nodenum == N3 && ScanMode == is_Back && Clue_Stage[1] == 5))
					{
						Change_Head = 1;
					}
					else if(nodesr.nowNode.nodenum == N7 && nodesr.nextNode.nodenum == B8)
					{
						Change_Head = 0;
					}
					else if(Change_Route == 1)
					{
						Change_Head = 0;
						Change_Route = 0;
					}
					else if(fabsf((need2turn(now_angle,nodesr.nextNode.angle))) > fabsf((need2turn(back_angle,nodesr.nextNode.angle))))
					{
						Change_Head = 1;
					}
					else
					{
						Change_Head = 0;
					}
					
					if(Change_Head == 1)
					{
						motor_all.Gspeed = Stop_T_Speed;
						angle.AngleG = getAngleZ();
						pid_mode_switch(is_Gyro);
						num=motor_all.Distance;
						
						if(nodesr.nowNode.nodenum == N3 && nodesr.nextNode.nodenum == P3)
						{
							while(fabsf(motor_all.Distance - num) < 20*60.0f)
							{
								vTaskDelay(2);
							}
						}
						else if(nodesr.nowNode.nodenum == N20 && nodesr.nextNode.nodenum == P7)     
						{
							while(fabsf(motor_all.Distance - num) < 30*60.0f) 
							{
								vTaskDelay(2);
							}
						}
						else if(nodesr.nowNode.nodenum == N3 && nodesr.nextNode.nodenum == N10)
						{
							while(fabsf(motor_all.Distance - num) < 15*60.0f) 
							{
								vTaskDelay(2);
							}
						}
						else
						{
							while(fabsf(motor_all.Distance - num) < 25*60.0f) 
							{
								vTaskDelay(2);
							}
						}
						
						struct PID_param origin_param = gyroT_pid_param;
						char oriGmax = motor_all.GyroT_speedMax;
						
						if(nodesr.lastNode.nodenum == C4 && nodesr.nowNode.nodenum == N20 && nodesr.nextNode.nodenum == P7)
							motor_all.GyroT_speedMax=25;
						else
							motor_all.GyroT_speedMax=22;//20  //25
						
						gyroT_pid_param.kp = 1.5;//0.65 //0.75
						gyroT_pid_param.ki = 0.0004/*66*/;//0.004  //0.5  //15   //30//0.00005
						gyroT_pid_param.kd = 0;
						
						float Reverse_angle = nodesr.nowNode.angle;
						//如果要换头-->is turn
						if(ScanMode == is_Front)
						{
							MODE_Switch(is_Back);
						}else
						{
							MODE_Switch(is_Front);
						}
						Reverse_angle = Modified_Angle(Reverse_angle,Reverse_angle);
						nodesr.nowNode.angle += 180;
						if(nodesr.nowNode.angle > 180)
						{
							nodesr.nowNode.angle -= 360;
						}
						mpuZreset(imu.yaw,nodesr.nowNode.angle);
						
						angle.AngleT = nodesr.nextNode.angle;
						pid_mode_switch(is_Turn);
						while(fabsf(need2turn(nodesr.nextNode.angle,getAngleZ()))>1.5f)
						{
							getline_error();
							if(Scaner.lineNum==1&&((Scaner.detail&0x180)!=0)&&(fabs(need2turn(nodesr.nextNode.angle,getAngleZ()))<fabs(need2turn(nodesr.nextNode.angle,nodesr.nowNode.angle))*0.15f))
							{
								break;
							}
							vTaskDelay(2);
						}
						mpuZreset(imu.yaw,nodesr.nextNode.angle);
						gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
						motor_all.GyroT_speedMax = oriGmax;
						gyroT_pid_param = origin_param;
						Change_Head = 0;
					}
					else if(nodesr.nowNode.flag&STOPTURN)//STOPTURN标志位待变原地转
					{
						motor_all.Gspeed = Stop_T_Speed;
						angle.AngleG = getAngleZ();
						pid_mode_switch(is_Gyro);
						num=motor_all.Distance;
						if(nodesr.nowNode.nodenum == N2 && nodesr.nextNode.nodenum == 	P2)
						{
							while(fabsf(motor_all.Distance-num) < 28*60.0f)
							{
								vTaskDelay(2);
							}
	
						}
						else if((nodesr.nowNode.nodenum == N7 && nodesr.nextNode.nodenum == P5)
							|| (nodesr.nowNode.nodenum == N7 && nodesr.nextNode.nodenum == B8))
						{
							while(fabsf(motor_all.Distance-num) < 22*60.0f)
							{
								vTaskDelay(2);
							}
	
						}
						else
						{
							while(fabsf(motor_all.Distance-num) < 22*60.0f)
							{
								vTaskDelay(2);
							}
						}
						struct PID_param origin_param = gyroT_pid_param;
						char oriGmax = motor_all.GyroT_speedMax;
						
						motor_all.GyroT_speedMax=25;
						gyroT_pid_param.kp = 0.65;//0.65
						gyroT_pid_param.ki = 0.0004/*66*/;//0.004  //0.5  //15   //30//0.00005
						gyroT_pid_param.kd = 0;
						
						angle.AngleT = nodesr.nextNode.angle;  //转绝对角度	
						pid_mode_switch(is_Turn);
						while(fabs(need2turn(angle.AngleT,getAngleZ()))>2)
						{   //角度达到要求
							vTaskDelay(2);
							getline_error();
							if(Scaner.lineNum==1&&((Scaner.detail&0x180)!=0)&&(fabs(need2turn(angle.AngleT,getAngleZ()))<fabs(need2turn(angle.AngleT,nodesr.nowNode.angle))*0.1f))
							{  
								send_play_specified_command(33);
								break;
							}
						}
						send_play_specified_command(30);
						motor_all.GyroT_speedMax = oriGmax;
						gyroT_pid_param = origin_param;
					}
					else//不用换头就陀螺仪转
					{
						struct PID_param origin_parm=gyroG_pid_param;
						float origin_speedMax = motor_all.GyroG_speedMax;
						
						if(((nodesr.nowNode.nodenum == N4) && (nodesr.nextNode.nodenum == B3))
							||((nodesr.nowNode.nodenum == N3) && (nodesr.nextNode.nodenum == N4)))
						{
							gyroG_pid_param.kp = 1.4;//2
							gyroG_pid_param.ki= 0.004;
							gyroG_pid_param.kd = 0;
							motor_all.GyroG_speedMax=50;
						}
						else if((nodesr.nowNode.nodenum == N1 && nodesr.nextNode.nodenum == B2)
							  ||(nodesr.nowNode.nodenum == N4)
							  ||(nodesr.nowNode.nodenum == N3)
							  ||(nodesr.nowNode.nodenum == N6 && nodesr.nextNode.nodenum == S2))
						{
							gyroG_pid_param.kp = 1.2;//2
							gyroG_pid_param.ki= 0.004;
							gyroG_pid_param.kd = 0;
							motor_all.GyroG_speedMax=50;
						}
						else if(nodesr.nowNode.nodenum == N1 && nodesr.nextNode.nodenum == P1)
						{
							gyroG_pid_param.kp = 0.95;//2
							gyroG_pid_param.ki = 0.004;
							gyroG_pid_param.kd = 0;
							motor_all.GyroG_speedMax=50;
						}
						else
						{
							if(ScanMode == is_Front)
							{
								gyroG_pid_param.kp=1.05;//1.25 //1.4//1.6 //0.85
								gyroG_pid_param.ki=0.004;
								gyroG_pid_param.kd=0;
								motor_all.GyroG_speedMax=50;
							}
							else
							{
								gyroG_pid_param.kp=0.75;//1.25 //1.4//1.6 //0.85 //0.8
								gyroG_pid_param.ki=0.004; 
								gyroG_pid_param.kd=0;
								motor_all.GyroG_speedMax=50;
							}
						}
						
						mpuZreset(imu.yaw, nodesr.nowNode.angle); // 陀螺仪校正
						angle.AngleG = nodesr.nowNode.angle;
						pid_mode_switch(is_Gyro);
						num = motor_all.Distance;
						if(nodesr.lastNode.nodenum == N13 && nodesr.nowNode.nodenum == N12 && nodesr.nextNode.nodenum == N8)
						{
							while(fabsf(motor_all.Distance - num) < 7*60)
							{
								vTaskDelay(2);
							}
						}
						else if(nodesr.lastNode.nodenum == N11 && nodesr.nowNode.nodenum == N10 && nodesr.nextNode.nodenum == N3)
						{
							while(fabsf(motor_all.Distance - num) < 5*60)
							{
								vTaskDelay(2);
							}
						}
						else if((nodesr.lastNode.nodenum == N4 && nodesr.nowNode.nodenum == N5 && nodesr.nextNode.nodenum == N12)
							|| (nodesr.lastNode.nodenum == N4 && nodesr.nowNode.nodenum == N3 && nodesr.nextNode.nodenum == N10))
						{
							while(fabsf(motor_all.Distance - num) < 5*60)
							{
								vTaskDelay(2);
							}
						}
						else if(nodesr.lastNode.nodenum == N3 && nodesr.nowNode.nodenum == N10 && nodesr.nextNode.nodenum == N11)
						{
							while(fabsf(motor_all.Distance - num) < 5*60)
							{
								vTaskDelay(2);
							}
						}	
						else if(nodesr.nowNode.nodenum == N3 && nodesr.nextNode.nodenum == S1)
						{
							while(fabsf(motor_all.Distance - num) < 4*60)
							{
								vTaskDelay(2);
							}
						}
						
						angle.AngleG = getAngleZ() + need2turn(nodesr.nowNode.angle,nodesr.nextNode.angle);
						if(angle.AngleG>180)	angle.AngleG -= 360;
						else if(angle.AngleG<=-180)	angle.AngleG += 360;
						
						motor_all.Gspeed = Gyro_Speed;
						pid_mode_switch(is_Gyro);
						
						while(fabsf(need2turn(getAngleZ(),angle.AngleG)) > 5) //2
						{
							getline_error();
							if(Scaner.lineNum==1 && (Scaner.detail&0x3C0) && (fabs(need2turn(angle.AngleG,getAngleZ()))<fabs(need2turn(angle.AngleG,nodesr.nowNode.angle))*0.25f))
							{
								break;
							}
							vTaskDelay(2);
						}
						gyroG_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
						gyroG_pid_param=origin_parm;
						motor_all.GyroG_speedMax=origin_speedMax;
					}
				}
			}
			//转完后进入循迹模式，更新结点，清空编码器值
			_flag=1;
			buzzer_off();
			pid_mode_switch(is_Line);
			nodesr.lastNode = nodesr.nowNode;
			nodesr.nowNode=nodesr.nextNode;	//更新结点
			motor_all.Cspeed=nodesr.nowNode.speed;
			nodesr.nextNode	= Node[getNextConnectNode(nodesr.nowNode.nodenum,route[map.point++])];//获取下一结点	
			if(nodesr.lastNode.nodenum == N10 && nodesr.nowNode.nodenum == N9 && nodesr.nextNode.nodenum == B9)
				nodesr.nextNode.angle = -140;
			scaner_set.EdgeIgnore=0;
			award_flag = 0;
			scaner_set.CatchsensorNum=0;
		    encoder_clear();
			LEFT_RIGHT_LINE = 0;
			mul2sing = 0;
			sing2mul = 0;
			nodesr.flag&=~0x04;		//	清除到达路口标志
		}
		else//如果路线走完
		{	
			motor_all.Cspeed = 0;
			motor_all.Gspeed = 0;
			LEFT_RIGHT_LINE = 0;
			award_flag = 0;
			CarBrake();
			vTaskDelay(2);
			_flag=1;
			map.routetime+=1;
		}	
	}	
	if(nodesr.flag&0x20)	//如果打到门是关的情况
	{		
		scaner_set.CatchsensorNum = 0;
		_flag=1;
		award_flag = 0;
		nodesr.nextNode	= Node[getNextConnectNode(nodesr.nowNode.nodenum,route[map.point++])];		//对下一结点进行更新
		if(nodesr.nextNode.nodenum != N8)//注意黄xx红的情况，第三个灯如果是==会在N8停（next==n3），所以是！=//且==的时候本来就是进door
		{
			nodesr.nextNode.function = DOOR;
		}
		nodesr.flag&=~0x20;
		motor_all.Cspeed=nodesr.nowNode.speed;
		pid_mode_switch(is_Line);
	}
	if(nodesr.flag&0x80)//如果打到门是开着
	{
		scaner_set.CatchsensorNum = 0;
		_flag=1;
		award_flag = 0;
		nodesr.nextNode	= Node[getNextConnectNode(nodesr.nowNode.nodenum,route[map.point++])];		//对下一结点进行更新
		nodesr.flag&=~0x80;
		encoder_clear();
		motor_all.Cspeed=nodesr.nowNode.speed;
		pid_mode_switch(is_Line);
	}
}
	
/**
 * @brief: 
 * @param {u8} fun
 * @return {*}
 */
void map_function(u8 fun)
{
	switch(fun)
	{
		case 0:break;
		case 1:break;		//寻线
		case UpStage      :Stage();break;//平台
		case BBridge  	  :Barrier_Bridge(150,4000);break;//长桥 长度，速度
		case BHill	  	  :Barrier_Hill(1);break;//楼梯
		case LBHill       :Barrier_Hill(2); break;  //双楼梯
		case SM           :Sword_Mountain();break;//刀山
		case View	      :view();break;//景点 后转
		case View1        :view1();break;//景点 直退
		case BACK         :back();break; 
		case BSoutPole	  :South_Pole(205);break;//南极
		case QQB	      :QQB_1();break;//跷跷板
		case BLBS         :Barrier_WavedPlate(87);break;//短减速板 速度，长度 80//85
		case BLBL	      :Barrier_WavedPlate(170);break;//长减速板 速度，长度//130改到150
		case DOOR	      :door();break;//打门
		case BHM          :Barrier_HighMountain(Low_Speed); break;//上珠峰
		case Scurve	  	  :S_curve();  break;
		case IGNORE       :ignore_node(); break;  //忽略该节点
		case UNDER        :undermou();break;
		case Special_node :Special_Node();break;
		default:break;		
	}
}


