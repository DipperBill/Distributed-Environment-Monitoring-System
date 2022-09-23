#ifndef _AHT20_H
#define _AHT20_H

#include "stm32f10x.h"  

/* 时钟端口、引脚定义 */
#define AHT20_SCL_PORT     GPIOB
#define AHT20_SCL_PIN      GPIO_Pin_4
#define AHT20_SDA_PORT     GPIOB
#define AHT20_SDA_PIN      GPIO_Pin_3
/* IO操作函数 */
#define AHT20_SCL_1    GPIO_SetBits(AHT20_SCL_PORT, AHT20_SCL_PIN)
#define AHT20_SCL_0    GPIO_ResetBits(AHT20_SCL_PORT, AHT20_SCL_PIN)
#define AHT20_SDA_1    GPIO_SetBits(AHT20_SDA_PORT, AHT20_SDA_PIN)
#define AHT20_SDA_0    GPIO_ResetBits(AHT20_SDA_PORT, AHT20_SDA_PIN)
#define AHT20_SDA_READ     GPIO_ReadInputDataBit(AHT20_SDA_PORT, AHT20_SDA_PIN)
/* IIC操作函数 */
void Delay_N10us(uint32_t t);
void SensorDelay_us(uint32_t t);
void DELAY_us(u16 t);
void Delay_1ms(uint32_t t);

void    AHT20_IIC_GPIO_INIT(void);
void    AHT20_SDA_OUT(void);
void    AHT20_SDA_IN(void);
void    AHT20_IIC_START(void);
void    AHT20_IIC_STOP(void);
void    AHT20_IIC_SendByte(u8 data);
u8 			AHT20_IIC_ReadByte(u8 ack);
u8 			AHT20_IIC_WaitACK(void);
void    AHT20_IIC_ACK(void);
void    AHT20_IIC_NACK(void);

u8 AHT20_Read_Status(void);
void AHT20_SendAC(void);
void AHT20_Read_CTdata(uint32_t *ct);
void AHT20_IIC_INIT(void);
void JH_Reset_REG(uint8_t addr);
void AHT20_Start_Init(void);

void AHT20_INIT(void);


#endif



