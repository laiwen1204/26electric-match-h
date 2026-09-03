#include "pid.h"

void PID_Init(PID_t *p)
{
    p->Target = 0;
    p->Actual = 0;
    p->Out = 0;
    p->Error0 = 0;
    p->Error1 = 0;
    p->ErrorInt = 0;
    p->FeedForward = 0;
    p->FeedForward2 = 0;
    p->IntSepThresh = 0;
    p->DeadZone = 0;
}

void PID_Update(PID_t *p)
{
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;

    /* 积分分离 + 抗饱和 */
    if (p->Ki != 0)
    {
        int allow_integral = 1;

        if (p->IntSepThresh > 0.0f)
        {
            if (p->Error0 > p->IntSepThresh || p->Error0 < -p->IntSepThresh)
                allow_integral = 0;
        }

        if (p->Out >= p->OutMax && p->Error0 > 0.0f) allow_integral = 0;
        if (p->Out <= p->OutMin && p->Error0 < 0.0f) allow_integral = 0;

        if (allow_integral)
            p->ErrorInt += p->Error0;
    }
    else
    {
        p->ErrorInt = 0;
    }

    /* PID计算: 前馈(二次) + P + I + D */
    {
        float abs_target = p->Target > 0.0f ? p->Target : -p->Target;
        p->Out = p->FeedForward * p->Target
               + p->FeedForward2 * p->Target * abs_target
               + p->Kp * p->Error0
               + p->Ki * p->ErrorInt
               + p->Kd * (p->Error0 - p->Error1);
    }

    /* 死区补偿 */
    if (p->DeadZone > 0.0f)
    {
        if (p->Out > 0.0f && p->Out < p->DeadZone)  p->Out = p->DeadZone;
        if (p->Out < 0.0f && p->Out > -p->DeadZone) p->Out = -p->DeadZone;
    }

    /* 输出限幅 */
    if (p->Out > p->OutMax) p->Out = p->OutMax;
    if (p->Out < p->OutMin) p->Out = p->OutMin;
}
