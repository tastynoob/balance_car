#include "ESPWifi.h"
#include "MYUSART.h"

typedef struct {
    float kp, ki, kd; // pid参数
    float point; //目标值
    float integral; //积分
    float derivative; //微分
    float previousError; //上次误差
} PID;

typedef enum {
    Backward = 0,
    Forward = 1,
} Dir;

#define TIM_MOT TIM3
#define TIM_IT TIM4
#define TIM_SUR TIM2

//最大占空比比较值
#define PWM_MAXDC 7000

extern bool ML_, MR_;

//电机方向速度控制
#define Mot_Ctrl(ml, mr)         \
    do {                         \
        if (ml >= 0) {           \
            TIM_MOT->CCR1 = ml;  \
            TIM_MOT->CCR2 = 0;   \
            ML_ = Forward;       \
        } else {                 \
            TIM_MOT->CCR1 = 0;   \
            TIM_MOT->CCR2 = -ml; \
            ML_ = Backward;      \
        }                        \
        if (mr >= 0) {           \
            TIM_MOT->CCR3 = mr;  \
            TIM_MOT->CCR4 = 0;   \
            MR_ = Forward;       \
        } else {                 \
            TIM_MOT->CCR3 = 0;   \
            TIM_MOT->CCR4 = -mr; \
            MR_ = Backward;      \
        }                        \
    } while (0)

//开启/关闭pid控制
#define PID_Cmd(mode) TIM_Cmd(TIM_IT, mode)

// PID速度控制
#define PID_SetPoint(ml, mr) \
    ml_pid.point = ml;       \
    mr_pid.point = mr

// 0.1ms计数器
#define clock TIM2->CNT

#define ml_tick (TIM_SUR->CCR1 - TIM_SUR->CCR2)

#define mr_tick (TIM_SUR->CCR3 - TIM_SUR->CCR4)

extern PID pd_balance; //平衡环控制
extern PID pi_speed; //速度环
extern PID pd_turn; //转向环

//初始化
void Mot_Init();

int PD_Balance();
int PI_Speed();
int PD_Turn();
void GetSpeed();

PID PIDStart(float k[3], float point);
float PID_Tuning(PID* pid, float realV);
