#ifndef ESPWIFI_H
#define ESPWIFI_H
#include "MYUSART.h"

#define wifi_tx tx2
#define wifi_rx rx2
#define wifi_en

#define SendU2(str) \
    SendString(USART2, str)

extern byte wifi_Buffer[];

bool WIFI_Init();

bool WIFI_SendString(char *str);

int WIFI_ReceiveString();

bool WIFI_WConnect();

#endif