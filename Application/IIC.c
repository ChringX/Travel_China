/*
 * @File: IIC.c
 * @Description: IIC驱动
 * @Version: 1.0.0
 * @Author:
 * @Date: 2023-09-13 20:33:36
 * @LastEditTime: 2023-09-15 15:56:25
 */
#include "iic.h"

/**
 * @brief:
 * @return {*}
 */
void IIC_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure;

	__HAL_RCC_GPIOD_CLK_ENABLE(); // 使能GPIOD时钟

	// PD12,PD13初始化设置
	GPIO_Initure.Pin = GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;   // 推挽输出
	GPIO_Initure.Pull = GPIO_PULLUP;		   // 上拉
	GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH; // 高速
	HAL_GPIO_Init(GPIOA, &GPIO_Initure);

	IIC_SDA_OUT;
	IIC_SCK_OUT;
}

/**
 * @brief:
 * @param {uint8_t} param
 * @return {*}
 */
void SDA(uint8_t param)
{
	GPIO_InitTypeDef GPIO_Initure = {0};
	if (param == 1)
	{
		GPIO_Initure.Pin = GPIO_PIN_11;
		GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;   // 推挽输出
		GPIO_Initure.Pull = GPIO_PULLUP;		   // 上拉
		GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH; // 高速
		HAL_GPIO_Init(GPIOA, &GPIO_Initure);
	}
	if (param == 0)
	{
		GPIO_Initure.Pin = GPIO_PIN_11;
		GPIO_Initure.Mode = GPIO_MODE_INPUT;	   // 输入
		GPIO_Initure.Pull = GPIO_PULLDOWN;		   // 下拉
		GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH; // 高速
		HAL_GPIO_Init(GPIOA, &GPIO_Initure);
	}
}

/**
 * @brief: 产生IIC起始信号
 * @return {*}
 */
void IIC_Start(void)
{
	SDA(0x01); // sda线输出
	IIC_SDA_OUT;
	IIC_SCK_OUT;
	delay_us(4);
	IIC_SDA_DOWN; // START:when CLK is high,DATA change form high to low
	delay_us(4);
	IIC_SCK_DOWN; // 钳住I2C总线，准备发送或接收数据
}

/**
 * @brief: 产生IIC停止信号
 * @return {*}
 */
void IIC_Stop(void)
{
	SDA(0x01); // sda线输出
	IIC_SCK_DOWN;
	IIC_SDA_DOWN; // STOP:when CLK is high DATA change form low to high
	delay_us(4);
	IIC_SCK_OUT;
	IIC_SDA_OUT; // 发送I2C总线结束信号
	delay_us(4);
}

/**
 * @brief: 等待应答信号到来
 * @return {uint8_t} 1，接收应答失败 0，接收应答成功
 */
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime = 0;
	SDA(0X00); // SDA设置为输入
	IIC_SDA_OUT;
	delay_us(1);
	IIC_SCK_OUT;
	delay_us(1);
	while (READ_SDA)
	{
		ucErrTime++;
		if (ucErrTime > 250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCK_DOWN; // 时钟输出0
	return 0;
}

/**
 * @brief: 产生ACK应答
 * @return {*}
 */
void IIC_Ack(void)
{
	IIC_SCK_DOWN;
	SDA(0X1);
	IIC_SDA_DOWN;
	delay_us(2);
	IIC_SCK_OUT;
	delay_us(2);
	IIC_SCK_DOWN;
}

/**
 * @brief: 不产生ACK应答
 * @return {*}
 */
void IIC_NAck(void)
{
	IIC_SCK_DOWN;
	SDA(0X01);
	IIC_SDA_OUT;
	delay_us(2);
	IIC_SCK_OUT;
	delay_us(2);
	IIC_SCK_DOWN;
}

// 返回从机有无应答
// 1，有应答
// 0，无应答
/**
 * @brief: IIC发送一个字节
 * @param {uint8_t} txd
 * @return {*}
 */
void IIC_Send_Byte(uint8_t txd)
{
	uint8_t t;
	SDA(0x01);
	IIC_SCK_DOWN; // 拉低时钟开始数据传输
	for (t = 0; t < 8; t++)
	{
		if ((txd & 0x80) >> 7)
		{
			IIC_SDA_OUT;
		}
		else
		{
			IIC_SDA_DOWN;
		}

		txd <<= 1;
		delay_us(2); // 对TEA5767这三个延时都是必须的
		IIC_SCK_OUT;
		delay_us(2);
		IIC_SCK_DOWN;
		delay_us(2);
	}
}

/**
 * @brief: 读1个字节，ack=1时，发送ACK，ack=0，发送NAck
 * @return {uint8_t}
 */
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i, receive = 0;
	SDA(0x0); // SDA设置为输入
	for (i = 0; i < 8; i++)
	{
		IIC_SCK_DOWN;
		delay_us(2);
		IIC_SCK_OUT;
		receive <<= 1;
		if (READ_SDA)
			receive++;
		delay_us(1);
	}
	if (!ack)
		IIC_NAck(); // 发送NAck
	else
		IIC_Ack(); // 发送ACK
	return receive;
}
