#ifndef _SPI_H
#define _SPI_H

#include "stm32f10x.h"
#include "main.h"		// 其中定义了NODE变量，指定主节点或子节点
/*
	子节点的SCK端口为PA3，主节点的SCK端口为PB10
*/
#if NODE
    #define sck_port	GPIOA
    #define sck_pin	GPIO_Pin_3
#else
    #define sck_port	GPIOB
    #define sck_pin	GPIO_Pin_10
#endif
/*
	其余SPI端口一致，定义如下
*/
#define miso_port 	GPIOA
#define miso_pin	GPIO_Pin_4
#define mosi_port 	GPIOA
#define mosi_pin	GPIO_Pin_5
#define nss_port 	GPIOA
#define nss_pin		GPIO_Pin_6
#define reset_port 	GPIOA
#define reset_pin	GPIO_Pin_7
/*
	SCK端口输出高低电平
*/
#define SPI_SCK_1    GPIO_SetBits(sck_port, sck_pin)		/* SCK = 1 */
#define SPI_SCK_0    GPIO_ResetBits(sck_port, sck_pin)		/* SCK = 0 */
/*
	MOSI端口输出高低电平
*/
#define SPI_MOSI_1    GPIO_SetBits(mosi_port, mosi_pin)		/* MOSI = 1 */
#define SPI_MOSI_0    GPIO_ResetBits(mosi_port, mosi_pin)	/* MOSI = 0 */
/*
	NSS端口输出高低电平
*/
#define SPI_NSS_1    GPIO_SetBits(nss_port, nss_pin)		/* nss = 1 */
#define SPI_NSS_0    GPIO_ResetBits(nss_port, nss_pin)		/* nss = 0 */
/*
	读取MISO端口电平
*/
#define SPI_READ_MISO    GPIO_ReadInputDataBit(miso_port, miso_pin)

void 	SPI_IoInit(void);						// SPI端口初始化
u8 		SPI_ReadWriteByte(u8 txData);		// SPI读取和写入数据(双向)
u8 		SPI_ReadByte(void);						// SPI读取数据
void 	SPI_WriteByte(u8 txData);			// SPI写入数据
	
#endif

