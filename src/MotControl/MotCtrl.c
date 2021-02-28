#include "MotCtrl.h"
#include "GY521.h"
#include "MYTIMER.h"

int16_t ML_pp = 0;
int16_t MR_pp = 0;

PID pd_balance; //平衡环控制
PID pi_speed;   //速度环
PID pd_turn;    //转向环

bool ML_ = Forward, MR_ = Forward;

void Mot_Init()
{
    MYGPIO_ClockOn(PC);
    MYGPIO_ClockOn(PB);
    MYGPIO_ClockOn(PA);
    MYGPIO_ModeSet(PA, 0, IPU, _in_);
    MYGPIO_ModeSet(PA, 1, IPU, _in_);
    MYGPIO_ModeSet(PB, 10, IPU, _in_);
    MYGPIO_ModeSet(PB, 11, IPU, _in_);

    MYGPIO_ModeSet(PC, 14, OUT_PP, _50MHz);
    MYGPIO_ModeSet(PC, 15, OUT_PP, _50MHz);

    //配置采样定时器中断1000
    MYTIM_Init(TIM4, 1000, 72 * 5); //200hz
    MYTIM_ITSetUP(TIM4, 1, 1);

    //配置电机控制PWM波 200hz
    MYPWM_Init(TIM3, 7200, 50);
    //Mot_Ctrl(0,0);

    pd_balance = PIDStart((float[3]){-380, -3.5, -24}, -1.75);
    pi_speed = PIDStart((float[3]){45, 1.5, 0}, 0);
    pd_turn = PIDStart((float[3]){200, 0, 100}, 0);

    {
        //配置速度测量定时器 输入捕获模式
        MYTIM_Init(TIM2, 65000, 7200); //0.1ms
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
        //PB10,11  PA0,1
        GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);
        TIM_ICInitTypeDef TIM_ICDef;
        MYGPIO_ModeSet(PA, 0, IPD, _in_);
        MYGPIO_ModeSet(PA, 1, IPD, _in_);
        MYGPIO_ModeSet(PB, 10, IPD, _in_);
        MYGPIO_ModeSet(PB, 11, IPD, _in_);

        TIM_ICDef.TIM_ICSelection = TIM_ICSelection_DirectTI;
        TIM_ICDef.TIM_ICPrescaler = TIM_ICPSC_DIV1;
        TIM_ICDef.TIM_ICFilter = 0;
        TIM_ICDef.TIM_Channel = TIM_Channel_1; //PA0
        TIM_ICDef.TIM_ICPolarity = TIM_ICPolarity_Rising;
        TIM_ICInit(TIM2, &TIM_ICDef);
        TIM_ICDef.TIM_Channel = TIM_Channel_2; //PA1
        TIM_ICDef.TIM_ICPolarity = TIM_ICPolarity_Falling;
        TIM_ICInit(TIM2, &TIM_ICDef);
        TIM_ICDef.TIM_Channel = TIM_Channel_3; //PB10
        TIM_ICDef.TIM_ICPolarity = TIM_ICPolarity_Rising;
        TIM_ICInit(TIM2, &TIM_ICDef);
        TIM_ICDef.TIM_Channel = TIM_Channel_4; //PB11
        TIM_ICDef.TIM_ICPolarity = TIM_ICPolarity_Falling;
        TIM_ICInit(TIM2, &TIM_ICDef);

        TIM_Cmd(TIM2, ENABLE);
    }
}

float angle_balance = 0;
float gyro_balance = 0;
int balance_dc = 0, velocity_dc = 0, turn_dc = 0;

int ml_speed = 0;
int mr_speed = 0;
float gyro_turn = 0;
#define makeup (1000 - 150)
//电机控制中断
void TIM4_IRQHandler(void)
{
    angle_balance = angle_z * 0.1 + angle_balance * 0.9;
    gyro_balance = gyro_y * 0.65 + gyro_balance * 0.35;
    gyro_turn = gyro_x * 0.1 + gyro_turn * 0.9;
    GetSpeed();

    balance_dc = PD_Balance();
    velocity_dc = PI_Speed();
    turn_dc = floorf(pd_turn.kp * pd_turn.point + pd_turn.kd * gyro_turn);

    balance_dc = balance_dc > 0 ? balance_dc + makeup : balance_dc - makeup;

    //////
    if (balance_dc > PWM_MAXDC)
        balance_dc = PWM_MAXDC;
    if (balance_dc < -PWM_MAXDC)
        balance_dc = -PWM_MAXDC;
    //////
    ML_pp = balance_dc + velocity_dc + turn_dc;
    MR_pp = balance_dc + velocity_dc - turn_dc;

    //printf("%f,%f,%d\n",angle_balance,gyro_balance,balance_dc);
    //printf("%d,%d\n", ml_speed, mr_speed);
    printf("%d,%d\n", ML_pp, MR_pp);
    //printf("%d\n",velocity_dc);

    Mot_Ctrl(ML_pp, MR_pp);
    TIM_IT->CNT = 0;
    TIM_ClearFlag(TIM_IT, TIM_FLAG_Update);
}

int last_t1 = 0, last_t2 = 0, last_t3 = 0, last_t4 = 0;

int t1 = 0, t2 = 0;
void GetSpeed()
{
    t1 = ml_tick;
    t2 = mr_tick;
    t1 = t1 > 0 ? t1 : -t1;
    t2 = t2 > 0 ? t2 : -t2;

    ml_speed = floorf((ML_ == Backward ? -1 : 1) * (1000.0 / t1) - 0.5) * 0.1 + ml_speed * 0.9;
    mr_speed = floorf((MR_ == Backward ? -1 : 1) * (1000.0 / t2) - 0.5) * 0.1 + mr_speed * 0.9;

    if (TIM_SUR->CNT > 50000)
    {
        TIM_SUR->CNT = 0;
    }
}

//平衡环控制
int PD_Balance()
{
    float error = (angle_balance - pd_balance.point) * 0.3 + pd_balance.previousError * 0.7;
    pd_balance.previousError = error;
    pd_balance.integral += error;
    // if (-0.2 <= error <= 0.2)
    //     pd_balance.integral = 0;
    return floorf(pd_balance.kp * error + pd_balance.ki * pd_balance.integral + pd_balance.kd * gyro_balance);
}

//速度环控制
int PI_Speed()
{
    float error = (ml_speed + mr_speed) - pi_speed.point;
    error = error * 0.1 + pi_speed.previousError * 0.9;
    pi_speed.previousError = error;
    pi_speed.integral += error;

    if (pi_speed.integral > 1000)
    {
        pi_speed.integral = 1000;
    }
    if (pi_speed.integral < -1000)
    {
        pi_speed.integral = -1000;
    }

    // if ((-5 <= floorf(mr_speed) <= 5) && (-5 <= floorf(mr_speed) <= 5))
    //     pi_speed.integral = 0;

    return floorf(pi_speed.kp * error + pi_speed.ki * pi_speed.integral);
}

//var自变量
PID PIDStart(float k[3], float point)
{
    PID pid;
    pid.kp = k[0];
    pid.ki = k[1];
    pid.kd = k[2];
    pid.point = point;
    pid.integral = 0;
    pid.derivative = 0;
    return pid;
}

//pid：目标pid控制参数，realV因变量当前的值,var自变量
float PID_Tuning(PID *pid, float realV)
{
    float error = pid->point - realV;
    pid->integral += error;
    pid->derivative = (error - pid->previousError);
    pid->previousError = error;
    return pid->kp * error + pid->ki * pid->integral + pid->kd * pid->derivative;
}