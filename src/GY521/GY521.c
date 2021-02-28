#include "stm32f10x.h"
#include "GY521.h"

//**************************************
//向I2C设备写入一个字节数据
//**************************************
bool Single_WriteI2C(u8 REG_Address, u8 REG_data)
{
    IIC_Start();                //起始信号
    IIC_SendBits(SlaveAddress); //发送设备地址+写信号

    if (!IIC_WACK())
        return false;

    IIC_SendBits(REG_Address); //内部寄存器地址，

    if (!IIC_WACK())
        return false;

    IIC_SendBits(REG_data); //内部寄存器数据，

    if (!IIC_WACK())
        return false;

    IIC_End(); //发送停止信号
    return true;
}

//**************************************
//从I2C设备读取一个字节数据
//**************************************
bool Single_ReadI2C(u8 REG_Address, u8 *a)
{
    IIC_Start();                //起始信号
    IIC_SendBits(SlaveAddress); //发送设备地址+写信号
    if (!IIC_WACK())
        return false;
    IIC_SendBits(REG_Address); //发送存储单元地址，从0开始
    if (!IIC_WACK())
        return false;
    //IIC_End();
    IIC_Start();                    //起始信号
    IIC_SendBits(SlaveAddress + 1); //发送设备地址+读信号
    if (!IIC_WACK())
        return false;
    *a = IIC_ReadBits(0); //读出寄存器数据	0 	NACK   1  ACK
    IIC_End();            //停止信号
    return true;
}

//**************************************
//初始化MPU6050
//**************************************
bool GY521_Init(void)
{

    IIC_Init();
    Delayms(100);

    // Single_WriteI2C(PWR_MGMT_1, 0x00);	//解除休眠状态
    // Single_WriteI2C(SMPLRT_DIV, 0x07);
    // Single_WriteI2C(CONFIG, 0x06);
    // Single_WriteI2C(GYRO_CONFIG, 0x18);
    // Single_WriteI2C(ACCEL_CONFIG, 0x01);

    bool a = Single_WriteI2C(MPU_60X0_PWR_MGMT_1_REG_ADDR, MPU_60X0_RESET_REG_VALU);
    if (!a)
        return a;

    Delayms(1);
    a = Single_WriteI2C(MPU_60X0_PWR_MGMT_1_REG_ADDR, MPU_60X0_PWR_MGMT_1_REG_VALU);
    if (!a)
        return a;
    Delayms(1);
    a = Single_WriteI2C(MPU_60X0_USER_CTRL_REG_ADDR, MPU_60X0_USER_CTRL_REG_VALU);
    if (!a)
        return a;
    Delayms(1);
    a = Single_WriteI2C(MPU_60X0_SMPLRT_DIV_REG_ADDR, MPU_60X0_SMPLRT_DIV_REG_VALU);
    if (!a)
        return a;
    Delayms(1);
    a = Single_WriteI2C(MPU_60X0_CONFIG_REG_ADDR, MPU_60X0_CONFIG_REG_VALU);
    if (!a)
        return a;
    Delayms(1);
    a = Single_WriteI2C(MPU_60X0_GYRO_CONFIG_REG_ADDR, MPU_60X0_GYRO_CONFIG_REG_VALU);
    if (!a)
        return a;
    Delayms(1);
    a = Single_WriteI2C(MPU_60X0_ACCEL_CONFIG_REG_ADDR, MPU_60X0_ACCEL_CONFIG_REG_VALU);
    if (!a)
        return a;
    Delayms(1);
    a = Single_WriteI2C(MPU_60X0_FIFO_EN_REG_ADDR, MPU_60X0_FIFO_EN_REG_VALU);
    if (!a)
        return a;

    return a;
}

//**************************************
//合成数据
//**************************************
short GetData(u8 REG_Address)
{
    u8 H = 0, L = 0;
    if (!Single_ReadI2C(REG_Address, &H))
        return -1;

    if (!Single_ReadI2C(REG_Address + 1, &L))
        return -1;
    return (H << 8) + L; //合成数据
}
