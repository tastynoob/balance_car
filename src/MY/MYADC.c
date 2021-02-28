#include "MYADC.h"


void MYADC_Init(ADC_TypeDef* ADCx){

    if(ADCx == ADC1){
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
    }
    else if(ADCx == ADC2){
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2,ENABLE);
    }
    
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);//6分频12mhz
    ADC_DeInit(ADCx);//复位
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;//单通道采样模式
    ADC_InitStructure.ADC_ContinuousConvMode =DISABLE;//一次采样
    //软件触发
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;//单通道采样
    ADC_Init(ADCx,&ADC_InitStructure);
    ADC_Cmd(ADCx,ENABLE);
    if(ADCx == ADC1)//只有adc1有温度传感器通道
    ADC_TempSensorVrefintCmd(ENABLE);//开启内部温度传感器
    ADC_ResetCalibration(ADCx);
    while(ADC_GetResetCalibrationStatus(ADCx));
    ADC_StartCalibration(ADCx);
    while(ADC_GetCalibrationStatus(ADCx));

    //ADC_RegularChannelConfig(ADC1,ADC_Channel_8,1,ADC_SampleTime_239Cycles5);
    //ADC_RegularChannelConfig(ADC1,ADC_Channel_16,2,ADC_SampleTime_239Cycles5);//温度传感器通道
    //ADC_RegularChannelConfig(ADC1,ADC_Channel_17,3,ADC_SampleTime_239Cycles5);//内部采集电压通道


    // ErrorStatus HseStartUpStatus;
    // RCC_DeInit();
    // RCC_HSEConfig(RCC_HSE_ON);
    // HseStartUpStatus = RCC_WaitForHSEStartUp();
    // if(HseStartUpStatus == SUCCESS){
    //     FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    //     FLASH_SetLatency(FLASH_Latency_2);
    //     RCC_HCLKConfig(RCC_SYSCLK_Div1);
    //     RCC_PCLK2Config(RCC_HCLK_Div1);
    //     RCC_PCLK1Config(RCC_HCLK_Div2);
    //     RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9);
    //     RCC_PLLCmd(ENABLE);
    //     while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY)==RESET);
    //     RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    //     while(RCC_GetSYSCLKSource()!=0x08);
    //     RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_AFIO,ENABLE);
    //     RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    //     RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
    // }


    // DMA_InitTypeDef DMA_InitStructure;
    // DMA_DeInit(DMA1_Channel1);
    // DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    // DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)AD_Value[0];
    // DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    // DMA_InitStructure.DMA_BufferSize = 10*3;
    // DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    // DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    // DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    // DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    // DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    // DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    // DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    // DMA_Init(DMA1_Channel1,&DMA_InitStructure);

}

int MYADC_GetV(ADC_TypeDef* ADCx,uint8_t channel){
    ADC_RegularChannelConfig(ADCx,channel,1,ADC_SampleTime_239Cycles5);
    ADC_SoftwareStartConvCmd(ADCx,ENABLE);
    while(!ADC_GetFlagStatus(ADCx,ADC_FLAG_EOC));
    return ADC1->DR;
}