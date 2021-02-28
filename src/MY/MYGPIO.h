#pragma once
#define MYGPIO_H
//配置GPIO模式
#include "stm32f10x.h"
#include "system_stm32f10x.h"
#include "SysTick.h"
#include "stdio.h"
#include "stm32f10x_exti.h"



#define uchar unsigned char
#define byte unsigned char
#define ushort unsigned short
#define bool unsigned char
#define false 0
#define true !false



#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x02000000U + ((addr & 0xFFFFF) << 5U) + ((bitnum) << 2U))
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))
//IO口地址映射
#define GPIOA_ODR_Addr (GPIOA_BASE + 12)
#define GPIOB_ODR_Addr (GPIOB_BASE + 12)
#define GPIOC_ODR_Addr (GPIOC_BASE + 12)
#define GPIOD_ODR_Addr (GPIOD_BASE + 12)
#define GPIOE_ODR_Addr (GPIOE_BASE + 12)
#define GPIOF_ODR_Addr (GPIOF_BASE + 12)
#define GPIOG_ODR_Addr (GPIOG_BASE + 12)
#define GPIOA_IDR_Addr (GPIOA_BASE + 8)
#define GPIOB_IDR_Addr (GPIOB_BASE + 8)
#define GPIOC_IDR_Addr (GPIOC_BASE + 8)
#define GPIOD_IDR_Addr (GPIOD_BASE + 8)
#define GPIOE_IDR_Addr (GPIOE_BASE + 8)
#define GPIOF_IDR_Addr (GPIOF_BASE + 8)
#define GPIOG_IDR_Addr (GPIOG_BASE + 8)
//每个端口的输出输出
#define PAout(n) BIT_ADDR(GPIOA_ODR_Addr, n) //输出
#define PAin(n) BIT_ADDR(GPIOA_IDR_Addr, n)  //输入
#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr, n)
#define PBin(n) BIT_ADDR(GPIOB_IDR_Addr, n)
#define PCout(n) BIT_ADDR(GPIOC_ODR_Addr, n)
#define PCin(n) BIT_ADDR(GPIOC_IDR_Addr, n)
#define PDout(n) BIT_ADDR(GPIOD_ODR_Addr, n)
#define PDin(n) BIT_ADDR(GPIOD_IDR_Addr, n)
#define PEout(n) BIT_ADDR(GPIOE_ODR_Addr, n)
#define PEin(n) BIT_ADDR(GPIOE_IDR_Addr, n)
#define PFout(n) BIT_ADDR(GPIOF_ODR_Addr, n)
#define PFin(n) BIT_ADDR(GPIOF_IDR_Addr, n)
#define PGout(n) BIT_ADDR(GPIOG_ODR_Addr, n)
#define PGin(n) BIT_ADDR(GPIOG_IDR_Addr, n)



typedef enum
{
    PA = (int)GPIOA,
    PB = (int)GPIOB,
    PC = (int)GPIOC,
    PD = (int)GPIOD,
    PE = (int)GPIOE,
} MYGPIO_TYPE;
typedef enum
{
    AIN = 0x0,//模拟输入
    IN_FLOATING = 0x4,//浮空输入
    IPD = 0x8,//下拉输入
    IPU = 0x8,//上拉输入
    
    OUT_PP = 0x0, //推挽输出
    OUT_DD = 0x4,//开漏输出
    AF_PP = 0x8,//复用推挽输出
    AF_OD = 0xc,//复用开漏输出
} MYGPIO_Mode;
typedef enum
{
    _in_ = 0x0,
    _10MHz = 0x1,
    _2MHz = 0x2,
    _50MHz = 0x3,
} MYGPIO_Speed;

#define Delayms(ms) SysTick_DelaySync(ms)
#define Delayus(us) SysTick_DelaySyncUs(us)


//引脚模式设置
void MYGPIO_ClockOn(MYGPIO_TYPE type);
void MYGPIO_ModeSet(MYGPIO_TYPE type, uint8_t pin, MYGPIO_Mode mode, MYGPIO_Speed speed);



void MYGPIO_ITConfig(MYGPIO_TYPE Px,int8_t pin,int pre,int sub);




void EXTI0_IRQHandler();
void EXTI1_IRQHandler();
void EXTI2_IRQHandler();
void EXTI3_IRQHandler();
void EXTI4_IRQHandler();
void EXTI9_5_IRQHandler();
void EXTI15_10_IRQHandler();


