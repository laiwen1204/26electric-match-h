#include "app.h"
#include "delay.h"

/* 全局变量定义 */
CarCommand_t CarCommand = CAR_CMD_STOP;
CarMode_t CarMode = CAR_MODE_VOFA;
TaskID_t TaskID = TASK2_LAP;   /* 上电默认任务二(整圈), 待机下按PA15切换 */
uint16_t BaseSpeed = BASE_SPEED_DEFAULT;
float TrackSpeed = 29.0f;   /* 灰度+低通实测: 29档整圈15.63s零丢线一次停稳; 31档过弯压边线会误触发停车 */
float LinePos = 3.5f;
float Task6TargetCm = 3.0f;   /* 要求6默认停球位置: +3cm, 可用VOFA slider在线改 */
uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];

PID_t SpeedPID_L = {
    .Kp = 2.25f,
    .Ki = 0.05f,
    .Kd = 0.0f,
    .OutMax = 1000.0f,
    .OutMin = -1000.0f,
    .FeedForward = 7.0f,
    .FeedForward2 = 0.0f,
    .IntSepThresh = 30.0f,
    .DeadZone = 0.0f,
};
PID_t SpeedPID_R = {
    .Kp = 2.25f,
    .Ki = 0.05f,
    .Kd = 0.0f,
    .OutMax = 1000.0f,
    .OutMin = -1000.0f,
    .FeedForward = 7.0f,
    .FeedForward2 = 0.0f,
    .IntSepThresh = 30.0f,
    .DeadZone = 0.0f,
};
PID_t PosPID = {
    .Kp = 5.0f,
    .Ki = 0.0f,
    .Kd = 15.0f,
    .OutMax = 400.0f,
    .OutMin = -400.0f,
    .FeedForward = 0.0f,
    .IntSepThresh = 0.0f,
    .DeadZone = 0.0f,
};

float TargetSpeed_L = 0.0f;
float TargetSpeed_R = 0.0f;

static int16_t s_pulse[2] = {0};
float s_speed[2] = {0.0f};

/* VOFA指令解析 ParseVOFACommand 已移至 APP/vofa.c (vofa_rx_packet/vofa_rx_flag
 * 仍留在 usart.c, 与UART RX中断强耦合, 不宜搬动) */

/* 编码器更新（两轮）
 * 原始映射（验证基准）: 左轮=通道1(PB0/PB1)取反, 右轮=通道2(PB10/PB11)不反。
 * 架空前进时 s_speed[0]/[1] 应同为正。 */
void Encoder_UpdateAll(void)
{
    s_pulse[0] = -Encoder_Get(1);
    s_pulse[1] =  Encoder_Get(2);

    for (int i = 0; i < 2; i++)
    {
        s_speed[i] = (float)s_pulse[i];
    }
}

/*======== 速度环PID初始化 ========*/
void SpeedPID_Init(void)
{
    PID_Init(&SpeedPID_L);
    PID_Init(&SpeedPID_R);
    /* 自动调参定稿(2026-07-28, 架空标定, 目标区间0~70):
       稳态误差0.7~3.1, 超调<=6, 左右不对称<=2.1, 全区间无发散。
       注意: FF切勿再大(>=10会过冲40%, 配合大Ki经电源内阻耦合形成反相极限环!) */
    SpeedPID_L.Kp = 2.25f;
    SpeedPID_L.Ki = 0.05f;
    SpeedPID_L.OutMax = 1000.0f;
    SpeedPID_L.OutMin = -1000.0f;
    SpeedPID_L.FeedForward = 7.0f;
    SpeedPID_L.FeedForward2 = 0.0f;
    SpeedPID_L.IntSepThresh = 30.0f;
    SpeedPID_L.DeadZone = 0.0f;
    SpeedPID_R.Kp = 2.25f;
    SpeedPID_R.Ki = 0.05f;
    SpeedPID_R.OutMax = 1000.0f;
    SpeedPID_R.OutMin = -1000.0f;
    SpeedPID_R.FeedForward = 7.0f;
    SpeedPID_R.FeedForward2 = 0.0f;
    SpeedPID_R.IntSepThresh = 30.0f;
    SpeedPID_R.DeadZone = 0.0f;
}

/*======== 速度环闭环控制（20ms周期） ========*/
void Car_SpeedLoop_Control(void)
{
    /* 1. 根据遥控指令设定左右轮目标速度 */
    switch (CarCommand)
    {
        case CAR_CMD_STOP:
            TargetSpeed_L = 0.0f;
            TargetSpeed_R = 0.0f;
            SpeedPID_L.ErrorInt = 0.0f;
            SpeedPID_R.ErrorInt = 0.0f;
            break;
        case CAR_CMD_FORWARD:
            TargetSpeed_L = (float)BaseSpeed;
            TargetSpeed_R = (float)BaseSpeed;
            break;
        case CAR_CMD_BACK:
            TargetSpeed_L = -(float)BaseSpeed;
            TargetSpeed_R = -(float)BaseSpeed;
            break;
        case CAR_CMD_LEFT:
            TargetSpeed_L = -(float)BaseSpeed;
            TargetSpeed_R =  (float)BaseSpeed;
            break;
        case CAR_CMD_RIGHT:
            TargetSpeed_L =  (float)BaseSpeed;
            TargetSpeed_R = -(float)BaseSpeed;
            break;
        default:
            TargetSpeed_L = 0.0f;
            TargetSpeed_R = 0.0f;
            break;
    }

    /* 2. 左轮速度环 */
    SpeedPID_L.Target = TargetSpeed_L;
    SpeedPID_L.Actual = s_speed[0];
    PID_Update(&SpeedPID_L);

    /* 3. 右轮速度环 */
    SpeedPID_R.Target = TargetSpeed_R;
    SpeedPID_R.Actual = s_speed[1];
    PID_Update(&SpeedPID_R);

    /* 4. PID输出映射到电机 */
    int16_t out_l = (int16_t)SpeedPID_L.Out;
    int16_t out_r = (int16_t)SpeedPID_R.Out;

    if (out_l > 0) {
        Motor_L_Run_forward((uint16_t)out_l);
    } else if (out_l < 0) {
        Motor_L_Run_back((uint16_t)(-out_l));
    } else {
        Motor_L_Stop();
    }

    if (out_r > 0) {
        Motor_R_Run_forward((uint16_t)out_r);
    } else if (out_r < 0) {
        Motor_R_Run_back((uint16_t)(-out_r));
    } else {
        Motor_R_Stop();
    }
}

/* 开环遥控指令执行（无PID，直驱PWM，调试DRV8701/电机时使用）
 * 注意：BaseSpeed 在此处就是 PWM 占空比 CCR 值（VOFA slider 范围请设 0~999，
 * 对应占空比 0%~99.9%，PWM 频率约 4kHz）。MG513 启动死区大约在 CCR 100 以上，
 * 低于该值电机不会转属正常现象。 */
void Car_ExecuteRemoteCommand(void)
{
    uint32_t pwm = BaseSpeed;                   /* 直接控制 PWM 占空比 */
    if (pwm > 999U) pwm = 999U;

    /* 开环模式下同步 TargetSpeed 显示值（单位: PWM CCR 0~999, 仅供 VOFA 波形观察） */
    switch (CarCommand)
    {
        case CAR_CMD_FORWARD:
            TargetSpeed_L =  (float)BaseSpeed;
            TargetSpeed_R =  (float)BaseSpeed;
            car_forward((uint16_t)pwm);
            break;
        case CAR_CMD_BACK:
            TargetSpeed_L = -(float)BaseSpeed;
            TargetSpeed_R = -(float)BaseSpeed;
            car_back((uint16_t)pwm);
            break;
        case CAR_CMD_LEFT:
            TargetSpeed_L = -(float)BaseSpeed;
            TargetSpeed_R =  (float)BaseSpeed;
            car_turnleft((uint16_t)pwm);
            break;
        case CAR_CMD_RIGHT:
            TargetSpeed_L =  (float)BaseSpeed;
            TargetSpeed_R = -(float)BaseSpeed;
            car_turnright((uint16_t)pwm);
            break;
        case CAR_CMD_STOP:
        default:
            TargetSpeed_L = 0.0f;
            TargetSpeed_R = 0.0f;
            Motor_All_Stop();
            break;
    }
}

/*======== 位置环PID初始化 ========
 * 整定结论(数字量巡线, TrackSpeed=14): Kp=5, Kd=2 最优。
 * 数字质心位置是0.5步进的, Kd过大会把量化台阶放大成转向冲击
 * (Kd=4开始抖动, Kd=15直接甩出赛道); Kd=0则欠阻尼过弯甩尾。 */
void PosPID_Init(void)
{
    PID_Init(&PosPID);
    PosPID.Kp = 4.0f;
    PosPID.Kd = 3.0f;
    PosPID.OutMax = 400.0f;
    PosPID.OutMin = -400.0f;
}

/*======== 黑线中心计算: 纯数字量质心 ========
 * 灰度模块数字位: 看到黑线的探头=1(极性见 GRAYSCALE_ON_BLACK);
 * 看到线的探头位置取平均即为线中心, 所有探头都没看到 -> 返回 -1 丢线。
 * 注意: 细线卡在两个探头之间时会判丢线, 由丢线保持逻辑短暂续走。 */
float CalcLinePosition(const uint16_t *bits)
{
    int nbits = 0, bsum = 0;
    uint8_t i;

    for (i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++)
    {
        if (bits[i]) { nbits++; bsum += i; }
    }

    if (nbits > 0)
        return (float)bsum / (float)nbits;
    return -1.0f;
}

/*======== 寻迹模式控制（20ms周期, 数字量质心 + 丢线保持） ========*/
void Car_Track_Control(void)
{
    float steering;
    int16_t out_l, out_r;
    float pos = LinePos;
    static float   s_hold_pos = 3.5f;

    /* 纯寻迹模式: 永不自动停车。
     * 丢线(细线卡在探头间/线缝/短暂冲出)时保持最后已知位置继续走,
     * 由人工或串口指令停车。 */
    if (pos < 0.0f)
    {
        pos = s_hold_pos;
    }
    else
    {
        s_hold_pos = pos;
    }

    /* 正常巡线：位置环 + 限幅防反转 */
    PosPID.Target = 3.5f;
    PosPID.Actual = pos;
    PID_Update(&PosPID);
    steering = PosPID.Out;
    if (steering >  TrackSpeed) steering =  TrackSpeed;
    if (steering < -TrackSpeed) steering = -TrackSpeed;
    TargetSpeed_L = TrackSpeed - steering;
    TargetSpeed_R = TrackSpeed + steering;

    /* 限幅 */
    if (TargetSpeed_L > 1000.0f) TargetSpeed_L = 1000.0f;
    if (TargetSpeed_L < -1000.0f) TargetSpeed_L = -1000.0f;
    if (TargetSpeed_R > 1000.0f) TargetSpeed_R = 1000.0f;
    if (TargetSpeed_R < -1000.0f) TargetSpeed_R = -1000.0f;

    /* 速度环PID */
    SpeedPID_L.Target = TargetSpeed_L;
    SpeedPID_L.Actual = s_speed[0];
    PID_Update(&SpeedPID_L);

    SpeedPID_R.Target = TargetSpeed_R;
    SpeedPID_R.Actual = s_speed[1];
    PID_Update(&SpeedPID_R);

    /* PID输出映射到电机 */
    out_l = (int16_t)SpeedPID_L.Out;
    out_r = (int16_t)SpeedPID_R.Out;

    if (out_l > 0) {
        Motor_L_Run_forward((uint16_t)out_l);
    } else if (out_l < 0) {
        Motor_L_Run_back((uint16_t)(-out_l));
    } else {
        Motor_L_Stop();
    }

    if (out_r > 0) {
        Motor_R_Run_forward((uint16_t)out_r);
    } else if (out_r < 0) {
        Motor_R_Run_back((uint16_t)(-out_r));
    } else {
        Motor_R_Stop();
    }
}
