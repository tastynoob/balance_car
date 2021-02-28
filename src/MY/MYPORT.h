#ifndef MYPORT_H
#define MYPORT_H
#include "MYGPIO.h"










//IIC总线
#define IIC_PIN_SCL 11
#define IIC_PIN_SDA 12
#define IIC_SCL PAout(IIC_PIN_SCL)
#define IIC_SDA PAout(IIC_PIN_SDA)
#define IIC_READ_SDA PAin(IIC_PIN_SDA) 

#define time_delay 1

#define IIC_Init() {\
MYGPIO_ClockOn(PA);\
MYGPIO_ModeSet(PA,IIC_PIN_SCL,OUT_DD,_50MHz);\
MYGPIO_ModeSet(PA,IIC_PIN_SDA,OUT_DD,_50MHz);\
IIC_SCL=1;\
IIC_SDA=1;\
}


#define IIC_SDA_OUT() \
MYGPIO_ModeSet(PA,IIC_PIN_SDA,OUT_DD,_50MHz)

#define IIC_SDA_IN()\
MYGPIO_ModeSet(PA,IIC_PIN_SDA,IPU,_in_)



void IIC_Start();
void IIC_End();
void IIC_SendBits(byte bits);
bool IIC_WACK();
void IIC_ACK();
void IIC_NACK(void);
byte IIC_ReadBits(unsigned char ack);



#endif