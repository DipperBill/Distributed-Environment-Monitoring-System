#ifndef __USART_H
#define __USART_H 			   

#include "stm32f10x.h"
#include "stdio.h"

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收

void uart1_Init(u32 bound);
void uart2_Init(u32 bound);
void uart3_Init(u32 bound);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);

#endif
