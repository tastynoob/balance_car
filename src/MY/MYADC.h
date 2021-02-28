#ifndef MYADC_H
#define MYADC_H

#include "MYGPIO.h"
#include "stm32f10x_adc.h"



#define ADC1_ChannelTemp 16
#define ADC1_ChannelVrefint 17


//dac初始化
void MYADC_Init(ADC_TypeDef* ADCx);
//adc读取值0~4096
int MYADC_GetV(ADC_TypeDef* ADCx,uint8_t channel);





#endif