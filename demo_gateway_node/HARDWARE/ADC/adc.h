#ifndef _adc_H
#define _adc_H

#include "stm32f10x.h"

void ADCx_Init(void);
u16 Get_ADC_Value(u8 ch,u8 times);


#endif
