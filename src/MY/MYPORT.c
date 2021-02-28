#include "MYPORT.h"

void delay(int t){
	while(t--);
}




//iic主机发送起始信号
void IIC_Start(){
	IIC_SDA_OUT();     //sda线输出
	IIC_SDA=1;
	IIC_SCL=1;
	delay(time_delay);
	IIC_SDA=0;	  	  
	delay(time_delay);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
	delay(time_delay);
}
//iic主机发送结束信号
void IIC_End(){
	IIC_SDA_OUT();//sda线输出
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
	IIC_SCL=0;
	delay(time_delay);	
	IIC_SCL=1;//
	delay(time_delay);
	IIC_SDA=1;
	delay(time_delay);
}
//iic主机发送8位数据
void IIC_SendBits(byte bits){
	IIC_SDA_OUT(); 	    
	delay(time_delay);
    for(u8 t=0;t<8;t++)
    {           
        IIC_SDA=(bits&0x80)>>7;
        bits<<=1;
		IIC_SCL=1;
		delay(time_delay);
		IIC_SCL=0;	
	}
	delay(time_delay);
	//在发送完数据后scl永远为低电平
}
//iic主机等待接收ack应答信号
bool IIC_WACK(){
	int t=0;
	IIC_SDA_IN();
	IIC_SDA=1;
	IIC_SCL=1;
	while(IIC_READ_SDA){
		t++;
		if(t>10000){
			IIC_End();
			return false;
		}
	}
	delay(time_delay);
	IIC_SCL=0;
	
	return true;
}

//产生ack应答信号
void IIC_ACK(){
	IIC_SDA_OUT();
	IIC_SCL=0;
	IIC_SDA=0;
	delay(time_delay);
	IIC_SCL=1;
	delay(time_delay);
	IIC_SCL=0;
	delay(time_delay);
}
//产生非应答信号
void IIC_NACK(void){
	IIC_SDA_OUT();
	IIC_SCL=0;
	IIC_SDA=1;
	delay(time_delay);
	IIC_SCL=1;
	delay(time_delay);
	IIC_SCL=0;
	delay(time_delay);
}			



//读8位数据，ack=1时，发送ACK，ack=0，发送nACK   
byte IIC_ReadBits(unsigned char ack)
{
	unsigned char i,receive=0;
	IIC_SDA_IN();//SDA设置为输入
	delay(time_delay);
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay(time_delay);
		IIC_SCL=1;
        receive<<=1;
        if(IIC_READ_SDA)receive++;   
    }					 
    if (!ack)
        IIC_NACK();//发送nACK
    else
        IIC_ACK(); //发送ACK   
    return receive;
}
