#include "car.h"

/* ============================================================================
 * 逐飞 DRV8701E 驱动板输入极性配置
 * 板上 PWM/DIR 经三极管电平转换, EN 为"低电平有效":
 *   MCU 占空比 0%   -> EN 常高 -> 电机全速 (!)
 *   MCU 占空比 100% -> EN 常低 -> 电机停止
 * 所以电机有效占空比 = 999 - CCR。
 * 若换用不反相的驱动板, 把 DRV_EN_INVERT 改为 0 即可。
 * 若发 forward 实际后退, 说明 DIR 也被反相, 把 DRV_DIR_INVERT 改为 1。
 * ============================================================================ */
#define DRV_EN_INVERT   1
#define DRV_DIR_INVERT  0

/* ============================================================================
 * 电机左右通道交换适配
 * 当前硬件: 物理左电机接在驱动通道2 (PWM2=PA16/CC0, DIR2=PA12/PH_R),
 *           物理右电机接在驱动通道1 (PWM1=PA17/CC1, DIR1=PA14/PH_L)。
 * (换DRV8701板时电机端子左右接反; 开环无法察觉, 闭环双轮会互相"交换"速度)
 * 若日后把电机线插回正确位置, 把 MOTOR_LR_SWAPPED 改为 0 !
 * ============================================================================ */
#define MOTOR_LR_SWAPPED  1

#if MOTOR_LR_SWAPPED
#define MOTOR_L_CC      DL_TIMER_CC_0_INDEX
#define MOTOR_R_CC      DL_TIMER_CC_1_INDEX
#define MOTOR_L_PH_SEL  0                    /* 左电机方向 -> PH_R */
#define MOTOR_R_PH_SEL  1                    /* 右电机方向 -> PH_L */
#else
#define MOTOR_L_CC      DL_TIMER_CC_1_INDEX
#define MOTOR_R_CC      DL_TIMER_CC_0_INDEX
#define MOTOR_L_PH_SEL  1
#define MOTOR_R_PH_SEL  0
#endif

/* 设置电机"有效"占空比: speed 0~999, 已处理板级反相 */
static void Motor_SetDuty(uint32_t cc_index, uint16_t speed)
{
    uint16_t ccr;
    if (speed > 999) speed = 999;
#if DRV_EN_INVERT
    ccr = 999 - speed;
#else
    ccr = speed;
#endif
    DL_TimerA_setCaptureCompareValue(TIMA1, ccr, cc_index);
}

/* ph_sel: 1=PH_L引脚, 0=PH_R引脚 (注意是引脚选择, 不是左右语义) */
static void Motor_SetDir(uint8_t ph_sel, uint8_t dir)
{
#if DRV_DIR_INVERT
    dir ^= 1;
#endif
    if (ph_sel) PH_L_OUT(dir);
    else        PH_R_OUT(dir);
}

void Motor_Init(void)
{
    /* DRV8701: GPIO已在 SYSCFG_DL_init() 中初始化, 上电先置于停止状态 */
    Motor_SetDuty(DL_TIMER_CC_0_INDEX, 0);
    Motor_SetDuty(DL_TIMER_CC_1_INDEX, 0);
    DL_Timer_startCounter(TIMA1);
    PH_L_OUT(0);
    PH_R_OUT(0);
}

void Motor_L_Run_forward(uint16_t speed)
{
    Motor_SetDir(MOTOR_L_PH_SEL, 1);
    Motor_SetDuty(MOTOR_L_CC, speed);
}

void Motor_L_Run_back(uint16_t speed)
{
    Motor_SetDir(MOTOR_L_PH_SEL, 0);
    Motor_SetDuty(MOTOR_L_CC, speed);
}

void Motor_L_Stop(void)
{
    Motor_SetDuty(MOTOR_L_CC, 0);   /* 反相板: 输出常高 -> EN 常低 -> 滑行停止 */
    Motor_SetDir(MOTOR_L_PH_SEL, 0);
}

void Motor_R_Run_forward(uint16_t speed)
{
    Motor_SetDir(MOTOR_R_PH_SEL, 1);
    Motor_SetDuty(MOTOR_R_CC, speed);
}

void Motor_R_Run_back(uint16_t speed)
{
    Motor_SetDir(MOTOR_R_PH_SEL, 0);
    Motor_SetDuty(MOTOR_R_CC, speed);
}

void Motor_R_Stop(void)
{
    Motor_SetDuty(MOTOR_R_CC, 0);
    Motor_SetDir(MOTOR_R_PH_SEL, 0);
}

void Motor_All_Stop(void)
{
    Motor_L_Stop();
    Motor_R_Stop();
}

/* 整车运动：差速驱动，两轮同向同速 */
void car_forward(uint16_t speed)
{
    Motor_L_Run_forward(speed);
    Motor_R_Run_forward(speed);
}

void car_back(uint16_t speed)
{
    Motor_L_Run_back(speed);
    Motor_R_Run_back(speed);
}

/* 坦克式原地左转：左轮后退，右轮前进 */
void car_turnleft(uint16_t speed)
{
    Motor_L_Run_back(speed);
    Motor_R_Run_forward(speed);
}

/* 坦克式原地右转：左轮前进，右轮后退 */
void car_turnright(uint16_t speed)
{
    Motor_L_Run_forward(speed);
    Motor_R_Run_back(speed);
}
