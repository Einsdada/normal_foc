/**
 * @file    pid.c
 * @brief   PI 控制器实现（位置式 + 积分限幅）
 */
#include "pid.h"

void PID_Init(PID_Handle_t *pid, float kp, float ki, float out_min, float out_max)
{
    pid->kp       = kp;
    pid->ki       = ki;
    pid->out_min  = out_min;
    pid->out_max  = out_max;
    pid->integral = 0.0f;
    pid->out      = 0.0f;
}

float PID_Calc(PID_Handle_t *pid, float err)
{
    float i_term, out;

    /* 积分累加 + 积分限幅（防饱和：积分项不超输出限幅） */
    pid->integral += err;
    i_term = pid->ki * pid->integral;
    if (pid->ki != 0.0f)
    {
        if (i_term > pid->out_max)
        {
            i_term         = pid->out_max;
            pid->integral  = pid->out_max / pid->ki;
        }
        else if (i_term < pid->out_min)
        {
            i_term         = pid->out_min;
            pid->integral  = pid->out_min / pid->ki;
        }
    }
    else
    {
        pid->integral = 0.0f;   /* 纯比例：积分不累加 */
    }

    /* 输出限幅 */
    out = pid->kp * err + i_term;
    if (out > pid->out_max)
        out = pid->out_max;
    else if (out < pid->out_min)
        out = pid->out_min;

    pid->out = out;
    return out;
}
