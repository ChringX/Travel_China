/*
 * @Rudder_control.c
 * @Description: 
 * @Version: 1.0.0
 * @Author: 
 * @Date: 2023-09-13 20:33:36
 * @LastEditTime: 2023-09-15 16:28:07
 */
#include "Rudder_control.h"
#include "uart.h"
//#include  "uart.h"
// iic
void Rudder_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	IIC_Init();//IIC初始化
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);//OE使能
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	Rudder_WriteOneByte(0x0,0x1);//响应0x70通用iic地址并且关闭sleep模式
//	设置频率
	uint8_t /*prescale,*/oldmode,newmode;
	oldmode = Rudder_ReadOneByte(0x0);
	newmode = (oldmode&0x7F) | 0x10; // sleep
	Rudder_WriteOneByte(0x0, newmode); // go to sleep
    Rudder_WriteOneByte(PRE_SCALE, 132); // set the prescaler
	Rudder_WriteOneByte(0x0,0x1);//响应0x70通用iic地址并且关闭sleep模式
	
//	int temp =1;
//	temp = Rudder_ReadOneByte(PRE_SCALE);
//	printf("%d\r\n",temp);
}

//在指定地址写入一个数据
//WriteAddr  :写入数据的目的地址    
//DataToWrite:要写入的数据
void Rudder_WriteOneByte(uint8_t WriteAddr,uint8_t DataToWrite)
{
	IIC_Start();  	
	IIC_Send_Byte(Rudder<<1);    //发送器件地址0x55,写数据 	 
	IIC_Wait_Ack();
    IIC_Send_Byte(WriteAddr);   //发送目标地址
	IIC_Wait_Ack();	
	IIC_Send_Byte(DataToWrite);//发送字节							   
	IIC_Wait_Ack();  		    	   
    IIC_Stop();//产生一个停止条件 
	delay_ms(10);	 
}
//指定地址读出一个数据
//ReadAddr:开始读数的地址  
//返回值  :读到的数据
uint8_t Rudder_ReadOneByte(uint8_t ReadAddr)
{				  
	uint8_t temp=0;		  	    																 
	IIC_Start();  
	IIC_Send_Byte(Rudder<<1);   //发送器件地址0X70,写数据 	   
	IIC_Wait_Ack();
	IIC_Send_Byte(ReadAddr);   //发送目标地址
	IIC_Wait_Ack();	
	IIC_Start();  	 	   
	IIC_Send_Byte((Rudder<<1)+1);           //进入接收模式			   
	IIC_Wait_Ack();	 
    temp=IIC_Read_Byte(0);		   
    IIC_Stop();					//产生一个停止条件	    
	return temp;
}
/*
	舵机控制
		Rudder_control(170, 0);	//机器人站立
		Rudder_control(310, 0);	//机器人躺下
		Rudder_control(360, 1); //右手举起
		Rudder_control(130, 1); //右手放下
		Rudder_control(150, 2); //左手举起
		Rudder_control(380, 2); //左手放下
		Rudder_control(380, 3); //头
*/
void Rudder_control(uint16_t aim,uint8_t id)
{
	Rudder_WriteOneByte(LED0_ON_L+(4*id),0x0);
	Rudder_WriteOneByte(LED0_ON_H+(4*id),0x0);
		
	Rudder_WriteOneByte(LED0_OFF_L+(4*id),(uint8_t)(aim&0xFF));
	Rudder_WriteOneByte(LED0_OFF_H+(4*id),(uint8_t)(aim>>8));
}

/*机器人动作*/
void Robot_Work(uint8_t id,uint8_t aim)
{
	if(id == BODY)
	{
		if(aim == UP)
			Rudder_control(170,0);
		else
			Rudder_control(310,0);
	}
	else if(id == RARM)
	{
		if (aim == UP)
			Rudder_control(360, 1);
		else
			Rudder_control(100, 1);
	}
	else if(id == LARM)
	{
		if (aim == UP)
			Rudder_control(100, 2);
		else
			Rudder_control(360, 2);
	}
}
