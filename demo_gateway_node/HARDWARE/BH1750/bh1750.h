#ifndef _bh1750_H
#define _bh1750_H

#include "stm32f10x.h"

/* 时钟端口、引脚定义 */
#define BH1750_SCL_PORT     GPIOB
#define BH1750_SCL_PIN      GPIO_Pin_11
#define BH1750_SDA_PORT     GPIOB
#define BH1750_SDA_PIN      GPIO_Pin_10
/* IO操作函数 */
#define BH1750_SCL_1    GPIO_SetBits(BH1750_SCL_PORT, BH1750_SCL_PIN)
#define BH1750_SCL_0    GPIO_ResetBits(BH1750_SCL_PORT, BH1750_SCL_PIN)
#define BH1750_SDA_1    GPIO_SetBits(BH1750_SDA_PORT, BH1750_SDA_PIN)
#define BH1750_SDA_0    GPIO_ResetBits(BH1750_SDA_PORT, BH1750_SDA_PIN)
#define BH1750_SDA_READ     GPIO_ReadInputDataBit(BH1750_SDA_PORT, BH1750_SDA_PIN)
/* IIC操作函数 */
void    BH1750_IIC_INIT(void);
void    BH1750_SDA_OUT(void);
void    BH1750_SDA_IN(void);
void    BH1750_IIC_START(void);
void    BH1750_IIC_STOP(void);
void    BH1750_IIC_SendByte(u8 data);
u8 			BH1750_IIC_ReadByte(u8 ack);
u8 			BH1750_IIC_WaitACK(void);
void    BH1750_IIC_ACK(void);
void    BH1750_IIC_NACK(void);
float BH1750_Read_data(void);


#endif

