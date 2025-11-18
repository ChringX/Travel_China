#ifndef _IIC_H_
#define _IIC_H_

#include "main.h"

/*×ó±ß:E12->SCK/E13->SDA£»ÓÒ±ß:E7->SCK/E8->SDA*/

#define out        0
#define in         1

#define yes        0
#define no         1

#define LEFT       0
#define RIGHT      1

#define RF          0
#define Gray        1

#define LEFT_SDA_H      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
#define LEFT_SDA_L      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
#define LEFT_SDA_read   HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13)
#define LEFT_SCL_H      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
#define LEFT_SCL_L      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
#define RIGHT_SDA_H     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
#define RIGHT_SDA_L     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
#define RIGHT_SDA_read  HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8)
#define RIGHT_SCL_H     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
#define RIGHT_SCL_L     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);

uint8_t I2C_WriteByte(uint8_t id,uint8_t SA,uint8_t RA,uint8_t data);
uint8_t I2C_ReadByte(uint8_t id,uint8_t SA,uint8_t RA,uint8_t *REG_data,uint8_t length);

void Gray_Init(void);
uint16_t Gray_GetLine(void);
void ScanerMode_Switch(uint8_t mode);

extern uint8_t ScanerMode;

#endif
