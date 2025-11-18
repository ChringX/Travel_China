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
					 
#define RESTMPUZ	 (1<<8)			    //陀螺仪校准
#define STOPTURN 	 (1<<9)				//停下来转弯
#define SLOWDOWN	 (1<<10)    		//减速

#define LEFT_LINE    (1<<11)   			//左循线(忽略右边白线干扰)
#define RIGHT_LINE	 (1<<12)   			//右循线

#define MCLEFT       (1<<13)
#define MCRIGHT      (1<<14)

#define DRIFT        (1<<15)    		//要用陀螺仪来转    		
#define L_follow     (1<<16)			//左循迹转弯（用来转弯
#define R_follow     (1<<17)			//右循迹转弯
#define MORELED      (1<<18)            //更多LED
#define LiuShui      (1<<19)			//流水灯
#define NOTURN       (1<<20)			//不转弯  

#define INGNORE      (1<<21)            //短直立景点后退
#define Temp_L		 (1<<22)			//临时左循迹
#define Temp_R		 (1<<23)			//临时右循迹
#define Temp_LiuShui (1<<24)			//临时流水

enum barriers {
	NONE = 1,
	UpStage,
	Bridge,
	Hill,
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
	IGNORE,
	UNDER,
	Special_node,
	DOOR1,
	UpStageP2
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
enum MapNode {	//MapNode
	S1, 	//0
	P1, 	//1
	N1,		//2
	B1,		//3
	B2,		//4
	B3,		//5
	N2,		//6
	P2,		//7
	S2,		//8
	P3,		//9
	N3, 	//10
	N4,		//11
	N5,		//12
	N6,		//13
	P4,		//14
	N7,		//15
	P5, 	//16
	B8, 	//17
	B9, 	//18
	N8, 	//19
	C1,		//20
	C2,		//21
	C3, 	//22
	N9, 	//23
	N10,	//24
	N12,	//25
	N13,	//26
	P6,		//27
	N14,	//28
	S3, 	//39
	S4,		//30
	N15,	//31
	S5,		//32
	C4,		//33
	C5, 	//34
	B4, 	//35
	B5, 	//36
	B6, 	//37
	B7, 	//38
	N16,	//49
	N18,	//40
	N19,	//41
	P7, 	//42
	N20,	//43
	N22,	//44
	C6,		//45
	C7, 	//46
	C8, 	//47
	C9,		//48
	P8,		//59
	N11,	//50
	G1		//51
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
	NODE lastNode;		//段起点
	NODE nowNode;		//段终点 - 要到达的节点
	NODE nextNode;		//段的后一结点
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

extern u8 Clue1route[50];
extern u8 Clue2route[50];
extern u8 Clue3route[50];
extern u8 Clue4route[50];
extern u8 Clue5route[50];
extern u8 Clue6route[50];
extern u8 Clue7route[50];
extern u8 Clue8route[50];
extern u8 Clue9route[50];
extern u8 Clue10route[50];
extern u8 Clue11route[50];
extern u8 Clue12route[50];
extern u8 Clue13route[50];
extern u8 Clue14route[50];
extern u8 Clue15route[50];
extern u8 Clue16route[50];
extern u8 Clue17route[50];
extern u8 Clue18route[50];
extern u8 Clue19route[50];
extern u8 Clue20route[50];
extern u8 Clue21route[50];
extern u8 Clue22route[50];
extern u8 Clue23route[50];
extern u8 Clue24route[50];
extern u8 Clue25route[50];
extern u8 Clue26route[50];
extern u8 Clue27route[50];
extern u8 Clue28route[50];
extern u8 Clue29route[50];
extern u8 Clue30route[50];
extern u8 Clue31route[50];
extern u8 Clue32route[50];
extern u8 Clue33route[50];
extern u8 Clue34route[50];
extern u8 Clue35route[50];
extern u8 Clue36route[50];
extern u8 Clue37route[50];
extern u8 Clue38route[50];
extern u8 Clue39route[50];
extern u8 Clue40route[50];
extern u8 Clue41route[50];
extern u8 Clue42route[50];
extern u8 Clue43route[50];
extern u8 Clue44route[50];
extern u8 Clue45route[50];
extern u8 Clue46route[50];
extern u8 Clue47route[50];
extern u8 Clue48route[50];
extern u8 Clue49route[50];
extern u8 Clue50route[50];
extern u8 Clue51route[50];
extern u8 Clue52route[50];
extern u8 Clue53route[50];
extern u8 Clue54route[50];
extern u8 Clue55route[50];
extern u8 Clue56route[50];
extern u8 Clue57route[50];
extern u8 Clue58route[50];
extern u8 Clue59route[50];
extern u8 Clue60route[50];
extern u8 Clue61route[50];
extern u8 Clue62route[50];
extern u8 Clue63route[50];
extern u8 Clue64route[50];
extern u8 Clue65route[50];
extern u8 Clue66route[50];
extern u8 Clue67route[50];
extern u8 Clue68route[50];
extern u8 Clue69route[50];
extern u8 Clue70route[50];
extern u8 Clue71route[50];
extern u8 Clue72route[50];
extern u8 Clue73route[50];
extern u8 Clue74route[50];
extern u8 Clue75route[50];
extern u8 Clue76route[50];
extern u8 Clue77route[50];
extern u8 Clue78route[50];
extern u8 Clue79route[50];
extern u8 Clue80route[50];
extern u8 Clue1P4route[50];
extern u8 Clue2P4route[50];
extern u8 Clue3P4route[50];
extern u8 Clue4P4route[50];
extern u8 Clue5P4route[50];
extern u8 Clue6P4route[50];
extern u8 Clue7P4route[50];
extern u8 Clue8P4route[50];
extern u8 Clue9P4route[50];
extern u8 Clue10P4route[50];
extern u8 Clue11P4route[50];
extern u8 Clue12P4route[50];
extern u8 Clue13P4route[50];
extern u8 Clue14P4route[50];
extern u8 Clue15P4route[50];
extern u8 Clue16P4route[50];
extern u8 Clue17P4route[50];
extern u8 Clue18P4route[50];
extern u8 Clue19P4route[50];
extern u8 Clue20P4route[50];
extern u8 Clue21P4route[50];
extern u8 Clue22P4route[50];
extern u8 Clue23P4route[50];
extern u8 Clue24P4route[50];
extern u8 Clue25P4route[50];
extern u8 Clue26P4route[50];
extern u8 Clue27P4route[50];
extern u8 Clue28P4route[50];
extern u8 Clue29P4route[50];
extern u8 Clue30P4route[50];
extern u8 Clue31P4route[50];
extern u8 Clue32P4route[50];
extern u8 Clue33P4route[50];
extern u8 Clue34P4route[50];
extern u8 Clue35P4route[50];
extern u8 Clue36P4route[50];
extern u8 Clue37P4route[50];
extern u8 Clue38P4route[50];
extern u8 Clue39P4route[50];
extern u8 Clue40P4route[50];
extern u8 Clue41P4route[50];
extern u8 Clue42P4route[50];
extern u8 Clue43P4route[50];
extern u8 Clue44P4route[50];
extern u8 Clue45P4route[50];
extern u8 Clue46P4route[50];
extern u8 Clue47P4route[50];
extern u8 Clue48P4route[50];
extern u8 Clue49P4route[50];
extern u8 Clue50P4route[50];
extern u8 Clue51P4route[50];
extern u8 Clue52P4route[50];
extern u8 Clue53P4route[50];
extern u8 Clue54P4route[50];
extern u8 Clue55P4route[50];
extern u8 Clue56P4route[50];
extern u8 Clue57P4route[50];
extern u8 Clue58P4route[50];
extern u8 Clue59P4route[50];
extern u8 Clue60P4route[50];
extern u8 Clue61P4route[50];
extern u8 Clue62P4route[50];
extern u8 Clue63P4route[50];
extern u8 Clue64P4route[50];
extern u8 Clue65P4route[50];
extern u8 Clue66P4route[50];
extern u8 Clue67P4route[50];
extern u8 Clue68P4route[50];
extern u8 Clue69P4route[50];
extern u8 Clue70P4route[50];
extern u8 Clue71P4route[50];
extern u8 Clue72P4route[50];
extern u8 Clue73P4route[50];
extern u8 Clue74P4route[50];
extern u8 Clue75P4route[50];
extern u8 Clue76P4route[50];
extern u8 Clue77P4route[50];
extern u8 Clue78P4route[50];
extern u8 Clue79P4route[50];
extern u8 Clue80P4route[50];
extern u8 TempRoute[50];
#endif







