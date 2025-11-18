#ifndef __MAP_H
#define __MAP_H
#include "sys.h"

#include "map_message.h"


#define NO      	 (1<<0) 
#define DLEFT 		 (1<<1)				//左边横线
#define DRIGHT 		 (1<<2)				//右边横线   右半边天
#define CLEFT	     (1<<3)				//左边斜线  左分岔路 45°
#define CRIGHT		 (1<<4)				//右边斜线
#define MUL2SING	 (1<<5)				//多条变一条
#define MUL2MUL 	 (1<<6) 		    //多条变多条
#define AWHITE  	 (1<<7)			   	//全黑
					 
#define RESTPID	     (1<<8)				//PID校准
#define RESTMPUZ	 (1<<9)			    //陀螺仪校准
#define STOPTURN 	 (1<<10)			//停下来转弯
#define SLOWDOWN	 (1<<11)    		//减速

#define LEFT_LINE    (1<<12)   			//左循线(忽略右边白线干扰)
#define RIGHT_LINE	 (1<<13)   			//右循线

#define MCLEFT       (1<<27)
#define MCRIGHT      (1<<14)

#define DRIFT        (1<<15)    		//要用陀螺仪来转    		
#define L_follow     (1<<16)			//左循迹转弯（用来转弯
#define R_follow     (1<<17)			//右循迹转弯
#define MORELED      (1<<18)            //更多LED
#define LiuShui      (1<<19)			//流水灯
#define NOTURN       (1<<20)			//不转弯  

#define INGNORE      (1<<22)            //短直立景点后退
#define Temp_L		 (1<<23)			//临时左循迹
#define Temp_R		 (1<<24)			//临时右循迹
#define Temp_LiuShui (1<<25)			//临时流水
#define Gyro_Turn 	 (1<<26)			//陀螺仪转弯速度

enum barriers {
	UpStage = 2,
	BBridge,
	BHill,
	LBHill,
	SM,
	View,
	View1,
	BACK,
	BSoutPole,
	QQB,
	BLBS,
	BLBL,
	DOOR,
	BHM,
	Scurve,
	IGNORE,
	UNDER,
	Special_node,
	DOOR1,
};

extern u8 route[100];
extern u8 door1route[100];
extern u8 door2route[100];
extern u8 door3route[100];
extern u8 door4route[100];
extern u8 door5route[100];
extern u8 door6route[100];
extern u8 door7route[100];
extern u8 door8route[100];
extern u8 door9route[100];
extern u8 door10route[100];
extern u8 door11route[100];
extern u8 door12route[100];
extern u8 door13route[100];
extern u8 door14route[100];
extern u8 Connect5Route[100];
extern u8 Connect6Route[100];

enum MapNode {	//MapNode
	S1, P1, N1,B1,B2,B3,N2, P2, S2,P3, N3, 
	N4, N5, N6, P4,N7,P5, B8, B9, N8, C1,
	C2,C3, N9, N10, N12, N13, P6,N14, S3, S4,
	N15, S5,C4, C5, B4, B5, B6, B7, N16, N18,
	N19,P7, N20, N22, C6,C7, C8, C9, P8,N11,
	G1
};

/**************************************/
//结点信息
//flag 0位寻线方式：0左寻线，1右寻线
//flag 123位到达路口标志：	000最左边打到，001最右边打到，010左边数线，011右边数线，100线数由多变成一条	
//flag 45位，数线数目	
//flag 6位，寻线方式是否要切换，1需要切换，0不需要切换
//flag 7位	需要陀螺仪校正
//flag 8~11	
typedef struct _node{
	u8 nodenum;     //结点名称
	u32  flag;	    //结点标志位
	float angle;	//角度	
	u16	step;		//线长
	float speed;	//寻线速度
	u8 function;    //结点函数
}NODE;

extern NODE Node[126];
/*************************/
//flag 0位：1编码器清零请求，0清零完毕
//flag 1位：启动路口判断
//flag 2位：是否到达路口
//flag 3位：arrive里temp清零
//flag 4位：Z轴置零
//flag 5位：路线处理复位 打到门
//flag 6位：没有门
//flag 7位：红灯
typedef struct _nodesr{
	u8 flag;
	NODE lastNode;		//当前结点
	NODE nowNode;		//当前结点
	NODE nextNode;	//下一结点
}NODESR;

extern uint8_t Change_Route;
extern NODESR nodesr;

struct Map_State {
	u8 point;
	u8 routetime;//第几次跑地图
};
extern struct Map_State map;
extern uint8_t Turn_Flag;

u8 getNextConnectNode(u8 nownode,u8 nextnode);
void mapInit(void);
void mapInit1(void);
void Cross(void);
void map_function(u8 fun);
u8 deal_arrive(void);

extern u8 Clue1route[100];
extern u8 Clue2route[100];
extern u8 Clue3route[100];
extern u8 Clue4route[100];
extern u8 Clue5route[100];
extern u8 Clue6route[100];
extern u8 Clue7route[100];
extern u8 Clue8route[100];
extern u8 Clue9route[100];
extern u8 Clue10route[100];
extern u8 Clue11route[100];
extern u8 Clue12route[100];
extern u8 Clue13route[100];
extern u8 Clue14route[100];
extern u8 Clue15route[100];
extern u8 Clue16route[100];
extern u8 Clue17route[100];
extern u8 Clue18route[100];
extern u8 Clue19route[100];
extern u8 Clue20route[100];
extern u8 Clue21route[100];
extern u8 Clue22route[100];
extern u8 Clue23route[100];
extern u8 Clue24route[100];
extern u8 Clue25route[100];
extern u8 Clue26route[100];
extern u8 Clue27route[100];
extern u8 Clue28route[100];
extern u8 Clue29route[100];
extern u8 Clue30route[100];
extern u8 Clue31route[100];
extern u8 Clue32route[100];
extern u8 Clue33route[100];
extern u8 Clue34route[100];
extern u8 Clue35route[100];
extern u8 Clue36route[100];
extern u8 Clue37route[100];
extern u8 Clue38route[100];
extern u8 Clue39route[100];
extern u8 Clue40route[100];
extern u8 Clue41route[100];
extern u8 Clue42route[100];
extern u8 Clue43route[100];
extern u8 Clue44route[100];
extern u8 Clue45route[100];
extern u8 Clue46route[100];
extern u8 Clue47route[100];
extern u8 Clue48route[100];
extern u8 Clue49route[100];
extern u8 Clue50route[100];
extern u8 Clue51route[100];
extern u8 Clue52route[100];
extern u8 Clue53route[100];
extern u8 Clue54route[100];
extern u8 Clue55route[100];
extern u8 Clue56route[100];
extern u8 Clue57route[100];
extern u8 Clue58route[100];
extern u8 Clue59route[100];
extern u8 Clue60route[100];
extern u8 Clue61route[100];
extern u8 Clue62route[100];
extern u8 Clue63route[100];
extern u8 Clue64route[100];
extern u8 Clue65route[100];
extern u8 Clue66route[100];
extern u8 Clue67route[100];
extern u8 Clue68route[100];
extern u8 Clue69route[100];
extern u8 Clue70route[100];
extern u8 Clue71route[100];
extern u8 Clue72route[100];
extern u8 Clue73route[100];
extern u8 Clue74route[100];
extern u8 Clue75route[100];
extern u8 Clue76route[100];
extern u8 Clue77route[100];
extern u8 Clue78route[100];
extern u8 Clue79route[100];
extern u8 Clue80route[100];
#endif







