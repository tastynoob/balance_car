#include "ESPWifi.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

byte wifi_Buffer[500] = {0};

bool IS_OK()
{
    int j = 10;
    while (j--)
    {
        WReceiveString(USART2, wifi_Buffer, 2000);
        printf(wifi_Buffer);
        if (strstr(wifi_Buffer, "OK"))
        {
            return true;
        }
    }
    return false;
}

bool WIFI_Init()
{
    MYUSART_Config(USART2, 115200);
    SendU2("AT+RST\r\n");
    Delayms(1000);
    SendU2("AT\r\n");
    if (IS_OK())
    {
        SendU2("AT+CIPMUX=1\r\n");
        if (!IS_OK())
            return false;
        Delayms(100);
        SendU2("AT+CIPSERVER=1,8080\r\n");
        if (!IS_OK())
            return false;
        return true;
    }
    else
    {
        return false;
    }
}

//等待用户连接
bool WIFI_WConnect()
{
    ReceiveString(USART2, wifi_Buffer);
    printf(wifi_Buffer);
    if (strstr(wifi_Buffer, "0,CONNECT"))
    {
        return true;
    }
    return false;
}

//wifi发送数据
bool WIFI_SendString(char *str)
{
    char len[10];
    sprintf(len, "%d", strlen(str));
    SendU2("AT+CIPSEND=0,");
    SendU2(len);
    SendU2("\r\n");
    SendU2(str);
    SendU2("\r\n");
    return IS_OK();
}

//wifi阻塞式接收数据
int WIFI_ReceiveString()
{
    return ReceiveString(USART2, wifi_Buffer);
}
