#ifndef __SX127X_HAL_H__
#define __SX127X_HAL_H__
#include "stm32f10x.h"	
#include "stdbool.h"
#include "stm32f10x_exti.h"

typedef enum
{
    RADIO_RESET_OFF,
    RADIO_RESET_ON,
}tRadioResetState;

//===================================SPIº¯ÊýÉùÃ÷===================================================
void SX127X_DIO0_INPUT(void);
void SX127X_DIO0_INTENABLE(void);
void SX127X_DIO0_INTDISABLE(void);
BitAction SX127X_DIO0_GetState(void);
void EXTI9_5_IRQHandler(void);
//void SX127X_DIO1_INPUT(void);
//void SX127X_DIO2_INPUT(void);
void SX127X_TXE_OUTPUT(BitAction PinState);
void SX127X_RXE_OUTPUT(BitAction PinState);
void SX127X_NSS_OUTPUT(BitAction PinState);
void SX127X_RESET_OUTPUT(BitAction PinState);
void SX127X_SPIGPIO_Init(void);
void SX127X_SPI_Init(void);
unsigned char SX127X_ReadWriteByte(unsigned char data);
void SX127X_WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );
void SX127X_ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );
#endif //__SX127X_HAL_H__
