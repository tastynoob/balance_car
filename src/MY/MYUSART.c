#include "MYUSART.h"
#include "string.h"


int fputc(int ch,FILE* f){
    USART_SendData(USART1,ch);
    while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
    return ch;
}

void MYUSART_Init(USART_TypeDef *USARTx)
{
    if (USARTx == USART1)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
        MYGPIO_ClockOn(PA);
        MYGPIO_ModeSet(PA,9, AF_PP, _50MHz);        //tx
        MYGPIO_ModeSet(PA,10, IN_FLOATING, _in_); //rx
    }
    else if (USARTx == USART2)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
        MYGPIO_ClockOn(PA);
        MYGPIO_ModeSet(PA,2, AF_PP, _50MHz);       //tx
        MYGPIO_ModeSet(PA,3, IN_FLOATING, _in_); //rx
    }
    else if (USARTx == USART3)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
        MYGPIO_ClockOn(PB);
        MYGPIO_ModeSet(PB,10, AF_PP, _50MHz);       //tx
        MYGPIO_ModeSet(PB,11, IN_FLOATING, _in_); //rx
    }
}
void MYUSART_ITSetOn(USART_TypeDef* USARTx,int pre,int sub){
    NVIC_InitTypeDef NVIC_def;
    if(USARTx == USART1){
        NVIC_def.NVIC_IRQChannel = USART1_IRQn;
    }
    if(USARTx==USART2){
        NVIC_def.NVIC_IRQChannel = USART2_IRQn;
    }
    if(USARTx==USART3){
        NVIC_def.NVIC_IRQChannel = USART3_IRQn;
    }
    NVIC_def.NVIC_IRQChannelPreemptionPriority=pre;
    NVIC_def.NVIC_IRQChannelSubPriority=sub;
    NVIC_def.NVIC_IRQChannelCmd=ENABLE;
    NVIC_Init(&NVIC_def);
    USART_ITConfig(USARTx,USART_IT_RXNE,ENABLE);
    USART_ITConfig(USARTx,USART_IT_ORE,DISABLE);
}
void MYUSART_Config(USART_TypeDef *USARTx, int baudRate)
{
    MYUSART_Init(USARTx);
    USART_InitTypeDef USART_Def;
    USART_Def.USART_BaudRate = baudRate;
    USART_Def.USART_WordLength = USART_WordLength_8b;
    USART_Def.USART_StopBits = USART_StopBits_1;
    USART_Def.USART_Parity = USART_Parity_No;
    USART_Def.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Def.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USARTx, &USART_Def);
    USART_Cmd(USARTx, ENABLE);
    USART_GetFlagStatus(USARTx, USART_FLAG_TC);
}

bool SendByte(USART_TypeDef *USARTx, byte data)
{
    int cnt = 0;
    USART_SendData(USARTx, data);
    //发送完成后发送标记位值1
    while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
    {
        cnt++;
        if (cnt > 100000)
        {
            return false;
        }
    }
    return true;
}

byte ReceiveByte(USART_TypeDef *USARTx)
{
    while (USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET)
        ;
    return USART_ReceiveData(USARTx);
}

void SendString(USART_TypeDef *USARTx, byte *str)
{
    int k = strlen((const char *)str);
    for (int i = 0; i < k; i++)
    {
        if (!SendByte(USARTx, str[i]))
            continue;
    }
}
int ReceiveString(USART_TypeDef *USARTx, byte *buffer)
{
    int len = 0;
    int i = 2000;
    while (!Available(USARTx));
    while (i--)
    {
        if (Available(USARTx))
        {
            buffer[len++] = USART_ReceiveData(USARTx);
            i = 2000;
        }
    }
    buffer[len] = 0;
    return len;
}

int WReceiveString(USART_TypeDef*USARTx,byte* buffer,int timeout){
    int len = 0;
    int i = 2000;
    int j = timeout;
    while (!Available(USARTx) && j--);
    while (i--)
    {
        if (Available(USARTx))
        {
            buffer[len++] = USART_ReceiveData(USARTx);
            i = 2000;
        }
    }
    buffer[len] = 0;
    return len;
}




