#ifndef __USART_H
#define __USART_H 			   

#include "stm32f10x.h"
#include "stdio.h"

#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

void uart1_Init(u32 bound);
void uart2_Init(u32 bound);
void uart3_Init(u32 bound);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);

#endif
