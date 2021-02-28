#include "MYGPIO.h"




void MYGPIO_ClockOn(MYGPIO_TYPE type){
    switch(type){
        case PA:
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
            break;
        case PB:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
            break;
        case PC:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);
            break;
    }
}


void MYGPIO_ModeSet(MYGPIO_TYPE type, uint8_t pin, MYGPIO_Mode mode, MYGPIO_Speed speed){
    int p = mode +  speed;
	if(mode==IPU){
		((GPIO_TypeDef*)type)->ODR |= (1<<pin);
	}
	else if(mode==IPD){
		((GPIO_TypeDef*)type)->ODR &= ~(1<<pin);
	}
    if(pin>7){
		pin=(pin-8)*4;
        p<<=(pin);
        ((GPIO_TypeDef*)type)->CRH &= ~((0x0000000f)<<(pin));
        ((GPIO_TypeDef*)type)->CRH |= p;
    }
    else{
        p <<=(pin*=4);
        ((GPIO_TypeDef*)type)->CRL &= ~((0x0000000f)<<(pin));
        ((GPIO_TypeDef*)type)->CRL |= p;
    }
}


void MYGPIO_ITConfig(MYGPIO_TYPE Px,int8_t pin,int pre,int sub){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    EXTI_InitTypeDef EXTI_Def;
    MYGPIO_ModeSet(Px,pin,IPU,_in_);
    if(Px == PA){
        GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,pin);
    }
    if(Px == PB){
        GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,pin);
    }
    if(Px == PC){
        GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,pin);
    }

    EXTI_Def.EXTI_Line = 1<<pin;
    EXTI_Def.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_Def.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿触发1->0
    EXTI_Def.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_Def);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    
    if(pin<5)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn + pin; //使能外部中断所在的通道
    else if(pin>4 && pin<10)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; //使能外部中断所在的通道
    else if(pin>9)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; //使能外部中断所在的通道


    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = pre; //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = sub; //子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //使能外部中断通道
    NVIC_Init(&NVIC_InitStructure); //根据结构体信息进行优先级初始化
}