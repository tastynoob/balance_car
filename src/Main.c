#include "ESPWifi.h"
#include "GY521.h"
#include "MYPORT.h"
#include "MotCtrl.h"
#include "math.h"
#include "string.h"

#define LED PBout(13)



void Init()
{
    SysTick_Init(72);
    
    MYGPIO_ClockOn(PA);
    MYGPIO_ClockOn(PB);
    MYGPIO_ModeSet(PB, 12, OUT_PP, _50MHz);
    MYGPIO_ModeSet(PB, 13, OUT_PP, _50MHz);
    PBout(12) = 1;

    MYUSART_Config(USART1, 115200);
    LED = 0;
    Delayms(500);
    Mot_Init();

    while (1) {
        LED = ~LED;
        if (GY521_Init()) {
            if (GetData(ACCEL_OUT)) {
                break;
            }
        }
        Delayms(100);
    }
    LED = 1;
    while (1) {
        LED = ~LED;
        if (WIFI_Init())
            break;
    }
    LED = 0;
}

int main(void)
{
    Init();
    PID_Cmd(ENABLE);
    //pi_speed.point = 20;
    Delayms(5000);
    //pi_speed.point = 0;
    pd_turn.point = 70;
    // Delayms(1000);
    // pd_turn.point = -5;
    while (!WIFI_WConnect())
        ;
    /*
    while (1) {
        WIFI_ReceiveString();
        LED = 1;
        Delayms(100);
        char* str;
        int a;
        if (str = strstr(wifi_Buffer, "forword")) {
            sscanf(str + 7, "%d", &a);
        }
        if (str = strstr(wifi_Buffer, "back")) {
            sscanf(str + 4, "%d", &a);
        }
        if (str = strstr(wifi_Buffer, "right")) {
            sscanf(str + 5, "%d", &a);
        }
        if (str = strstr(wifi_Buffer, "left")) {
            sscanf(str + 4, "%d", &a);
        }
        LED = 0;
    }
	*/
}
