// 八路灰度巡线传感器 (4051多路复用, GPIO扫描)
// AD0 -> PA22, AD1 -> PA24, AD2 -> PA7, OUT -> PA8
#include "ti_msp_dl_config.h"
#include "delay.h"
#include "usart.h"
#include "oled_software_i2c.h"
#include "car.h"
#include "encoder.h"
#include "pid.h"
#include "timer.h"
#include "app.h"
#include "vofa.h"

/* 回A点自动停车检测总开关:
 * 1 = 启用(比赛用); 0 = 屏蔽(纯巡迹调PID时用, 弯道不会误停) */
#define HOME_DETECT_EN   1

/* LinePos 一阶低通滤波系数(0~1):
 * 数字量位置是台阶跳变的, D项对跳变敏感引起微抖;
 * 滤波后位置平滑爬升, 转向更稳。越大响应越快/滤波越弱, 越小越平滑/延迟越大。
 * 0.4 ≈ 延迟1~2个控制拍(20~40ms), 巡线够用; 平衡球阶段若还抖可降到0.3 */
#define LINE_LPF_ALPHA   0.4f

/* >=4路停车确认时长(ms): 主循环高频全扫(约1ms一次), 持续亮满该时长才算压到A标记。
 * 1ms ≈ 单扫即停: 4路同亮物理上只有压A点宽标记才会出现(弯道最多1~2路),
 * 无需长确认; 高频采样保证短暂同亮窗口不再漏采 */
#define HOME_N4_HOLD_MS  1

/* 右3路停车确认时长(ms): 主循环高频采样, 持续亮满该时长才算压到A标记。
 * 30ms ≈ 1.5个控制拍; 弯道细线不会持续30ms停在同一侧 */
#define HOME_R3_HOLD_MS  30

/* 最右2路(6/7)停车确认时长(ms): 比右3路更灵敏, 提高高速过A的抓获率。
 * 20ms ≈ 1个控制拍; (曾试10ms: 右弯压边线有误停前科, 保持20ms) */
#define HOME_R2_HOLD_MS  20

/* 起跑屏蔽期(ms): 武装(驶离A点)后该时间内不进行任何停止线识别,
 * 防止起步段/前半程弯道特征误触发; 10s后才开启回A检测 */
#define HOME_BLIND_MS    10000

static uint32_t lap_start_ms = 0;    /* 驶离A点(武装)时刻, 圈速计时起点 */
static uint32_t last_lap_ms = 0;     /* 最近一圈用时(ms), 0=尚未完圈 */
static uint8_t  task_timed_started = 0;   /* 任务四/五计时起点已由模式切换沿初始化(防VOFA进TRACK瞬间用旧起点误停) */

/* ---- 按键扫描+定时器中断消抖(PA9发车/PA15切任务) ----
 * 在 TIMG7 1ms 中断里每毫秒采样电平: 连续低电平满60ms判定为一次有效按下,
 * 锁存到松开为止; ISR只置事件标志, 动作(OLED/串口/模式切换)在主循环执行。
 * (此前GPIO边沿中断方案在板子上不触发, 原因不明, 改为定时器扫描最稳妥) */
#define KEY_EVENT_PA9    0x01U
#define KEY_EVENT_PA15   0x02U
#define KEY_DEBOUNCE_MS  60U
static volatile uint8_t g_key_event = 0;

void Keys_ScanTick(void)   /* 由 TIMG7 1ms 中断调用, 见 SYSTEM/timer.c */
{
    static uint16_t cnt9 = 0, cnt15 = 0;
    static uint8_t  lat9 = 0, lat15 = 0;

    if (!(DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_9) & DL_GPIO_PIN_9))
    {
        if (cnt9 < KEY_DEBOUNCE_MS) cnt9++;
        if (!lat9 && cnt9 >= KEY_DEBOUNCE_MS)
        {
            lat9 = 1;
            g_key_event |= KEY_EVENT_PA9;
        }
    }
    else
    {
        cnt9 = 0;
        lat9 = 0;
    }

    if (!(DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_15) & DL_GPIO_PIN_15))
    {
        if (cnt15 < KEY_DEBOUNCE_MS) cnt15++;
        if (!lat15 && cnt15 >= KEY_DEBOUNCE_MS)
        {
            lat15 = 1;
            g_key_event |= KEY_EVENT_PA15;
        }
    }
    else
    {
        cnt15 = 0;
        lat15 = 0;
    }
}

/* 回A停车动作: 停车 + 定格圈速 + 上报 */
static void Home_Stop(uint8_t n_on, uint8_t r3, uint8_t r2)
{
    last_lap_ms = g_ms_tick - lap_start_ms;
    CarMode = CAR_MODE_VOFA;
    CarCommand = CAR_CMD_STOP;
    TargetSpeed_L = 0.0f;
    TargetSpeed_R = 0.0f;
    SpeedPID_L.ErrorInt = 0.0f;
    SpeedPID_R.ErrorInt = 0.0f;
    Motor_All_Stop();
    printf("[HOME] back to A, TIME %lu.%03lus (on=%u,r3=%u,r2=%u)\n",
           (unsigned long)(last_lap_ms / 1000),
           (unsigned long)(last_lap_ms % 1000),
           n_on, r3, r2);
}

/* 定时任务(任务四A->B / 任务5三角剖面)停车动作: 到时停车 + 定格运行时间 + 上报 */
static void TaskTimed_Stop(void)
{
    last_lap_ms = g_ms_tick - lap_start_ms;
    task_timed_started = 0;
    CarMode = CAR_MODE_VOFA;
    CarCommand = CAR_CMD_STOP;
    TargetSpeed_L = 0.0f;
    TargetSpeed_R = 0.0f;
    SpeedPID_L.ErrorInt = 0.0f;
    SpeedPID_R.ErrorInt = 0.0f;
    Motor_All_Stop();
    printf((TaskID == TASK4_AB) ? "[TASK4] A->B done, TIME %lu.%03lus\n"
       : (TaskID == TASK6_POS) ? "[TASK6] POS done, TIME %lu.%03lus\n"
                               : "[TASK5] TRI done, TIME %lu.%03lus\n",
           (unsigned long)(last_lap_ms / 1000),
           (unsigned long)(last_lap_ms % 1000));
}

int main(void)
{
    uint8_t slow_task_cnt = 0;
    uint8_t  home_armed = 0;      /* 回A检测武装标志(20ms拍与主循环共用) */
    uint32_t r3_on_ms = 0;        /* 右3路开始持续亮的时刻, 0=当前未亮 */
    uint32_t r2_on_ms = 0;        /* 最右2路开始持续亮的时刻, 0=当前未亮 */
    uint32_t n4_on_ms = 0;        /* >=4路同亮开始的时刻, 0=当前未满足 */

    SYSCFG_DL_init();

    /* 使能 DMA 中断（UART TX DMA 续传需要） */
    NVIC_ClearPendingIRQ(DMA_INT_IRQn);
    NVIC_EnableIRQ(DMA_INT_IRQn);

    USART_Init();
    printf("\n[BOOT]\n");   /* 上电立即给串口一个信号，便于主机尽早建立连接 */
    OLED_Init();   /* 上电清屏, OLED只显示计时(寻迹中实时, 停车后定格) */
    OLED_Printf(0, 3, 16, "TASK2:LAP ");   /* 上电默认任务二, PA15短按切换 */

    Motor_Init();
    Encoder_Init();
    Timer_Init();
    Grayscale_Sensor_Init();   /* 灰度模块: GPIO已由SysConfig配好, 此处为空函数 */

    /* 按键引脚 PA9(发车)/PA15(切任务): 输入+上拉,
     * 由 TIMG7 1ms 中断里的 Keys_ScanTick() 定时扫描消抖(见 empty.c) */
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM20,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM37,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    SpeedPID_Init();
    PosPID_Init();

    /* 上电默认TASK2: 主动通知STM32停止M2006(等其先上电初始化完再发, 发2次保险),
     * 防止STM32侧残留状态导致摆杆锁在某个角度 */
    delay_ms(300);
    Link_NotifyTaskStop();
    delay_ms(50);
    Link_NotifyTaskStop();

    /* 编码器四路 GPIOB 双边沿中断 */
    NVIC_EnableIRQ(GPIOB_INT_IRQn);

    while (1)
    {
        ParseVOFACommand();

        /* ---- 按键事件处理(GPIOA外部中断产生, 主循环执行动作) ----
         * PA9: 进入寻迹(仅小车类任务2/4/5); PA15: 待机下循环切换任务。
         * OLED/阻塞串口放主循环, 不占中断上下文。 */
        if (g_key_event)
        {
            static const char *const s_task_oled[5] = {
                "TASK2:LAP ", "TASK3:ROLL", "TASK4:A-B ",
                "TASK5:TRI ", "TASK6:POS "
            };
            static const uint8_t s_task_num[5] = { 2, 3, 4, 5, 6 };
            uint8_t ev;

            __disable_irq();            /* 取走事件标志(与ISR互斥) */
            ev = g_key_event;
            g_key_event = 0;
            __enable_irq();

            /* PA15: 待机(VOFA)下循环切换任务
             * TASK2: 要求2整圈高速回A计时停车(选中即发stop停M2006);
             * TASK3: 要求3滚球, 选中仅停止M2006待命, 按PA9开始键才发q3启动;
             * TASK4: 要求4 A->B匀加速起步到时自停(选中即稳O点);
             * TASK5: 要求5匀加速30s到时自停(选中即稳O点);
             * TASK6: 要求6行驶剖面同TASK5, 钢球稳在自定义点(选中即发q6:Task6TargetCm) */
            if ((ev & KEY_EVENT_PA15) && CarMode == CAR_MODE_VOFA)
            {
                TaskID = (TaskID_t)((TaskID + 1) % 5);
                OLED_Printf(0, 3, 16, (char *)s_task_oled[TaskID]);
                printf("[TASK] -> TASK%u\n", (unsigned)s_task_num[TaskID]);
                /* 切到TASK4/5立即稳O点(不等发车); 切到TASK6立即稳自定义点;
                 * 切到TASK2/TASK3只发stop停M2006待命(任务3由PA9开始键启动) */
                if (TaskID == TASK2_LAP || TaskID == TASK3_ROLL)
                    Link_NotifyTaskStop();
                else
                    Link_NotifyTaskStart(s_task_num[TaskID]);
            }

            /* PA9 开始键: 任务3 -> 发q3启动STM32滚球(小车不动);
             * 小车类任务2/4/5/6 -> 进入TRACK寻迹发车(任务6剖面同任务5) */
            if ((ev & KEY_EVENT_PA9) && CarMode == CAR_MODE_VOFA)
            {
                if (TaskID == TASK3_ROLL)
                {
                    printf("[TASK3] GO, send q3\n");
                    Link_NotifyTaskStart(3);
                }
                else if (TASK_IS_CAR(TaskID))
                {
                    PID_Init(&PosPID);
                    CarMode = CAR_MODE_TRACK;
                }
            }
        }

        /* ---- STM32 链路回复回显: 切任务后看这里确认真启动了还是被拒绝 ----
         * OLED第一行: LNK:OK=已启动 / LNK:REJ=被拒绝(原因看VOFA [STM32]行) */
        if (link_rx_flag)
        {
            char line[96];
            __disable_irq();
            strncpy(line, link_rx_line, sizeof(line) - 1);
            line[sizeof(line) - 1] = '\0';
            link_rx_flag = 0;
            __enable_irq();

            printf("[STM32] %s\n", line);
            if (CarMode == CAR_MODE_VOFA)
            {
                if (strstr(line, "started") != NULL)
                    OLED_Printf(0, 1, 16, "LNK:OK ");
                else if (strstr(line, "rejected") != NULL)
                    OLED_Printf(0, 1, 16, "LNK:REJ");
                else if (strstr(line, "stop") != NULL)
                    OLED_Printf(0, 1, 16, "LNK:STP");
            }
        }

        if (timer_flag)
        {
            timer_flag = 0;

            Encoder_UpdateAll();

            /* 巡线传感(纯数字量): 4051逐路扫描8个探头(共约0.8ms),
             * 1=看到黑线。看到线的探头取平均位置, 数字位直接进 OLED 显示。
             * 位置做一阶低通: 抹平数字量台阶跳变, 抑制D项引起的微抖;
             * 丢线帧(-1)不进滤波器, 避免污染平滑值。 */
            Grayscale_Sensor_Read_All(g_sensor_data);
            {
                static float s_fpos = 3.5f;
                static uint8_t s_lpf_init = 0;
                float raw = CalcLinePosition(g_sensor_data);
                if (raw >= 0.0f)
                {
                    if (!s_lpf_init) { s_fpos = raw; s_lpf_init = 1; }
                    else             { s_fpos += LINE_LPF_ALPHA * (raw - s_fpos); }
                    LinePos = s_fpos;
                }
                else
                {
                    LinePos = -1.0f;
                }
            }

            /* 按键处理已改为外部中断(GPIOA下降沿)+主循环事件, 见 while(1) 开头 */

#if HOME_DETECT_EN
            /* 任务二(要求2): 巡线一圈回到A点自动停车。
             * 停车判定全部在主循环高频采样里做(约1ms全扫一次, 比20ms拍密20倍,
             * 防止高速过A时"4路同亮窗口"落在两个控制拍之间被漏采), 满足其一即停:
             *   1) >=4路同亮持续10ms (正压A点宽标记, 弯道物理上不可能4路同亮);
             *   2) 最右2路(6/7)持续亮满20ms (压标记右侧);
             *   3) 右3路(5/6/7)持续亮满30ms (右斜压标记)。
             * 本拍只做"模式切换沿"与"武装":
             *   进入寻迹(按键或串口)时按当前任务初始化:
             *     TASK2: 速度29高速档, 复位回A检测, 圈速起点等武装(驶离A标记)才生效;
             *     TASK4/TASK5: 速度剖面起步(初速0), 不做武装/回A检测, 起点立即生效(到时自停)。
             *   武装: 连续3拍 <4路看到线(已驶离A标记) -> 武装+圈速起点;
             *   武装后 HOME_BLIND_MS(10s) 内屏蔽所有停车判定, 防前半程误触发。 */
            {
                static uint8_t s_arm_cnt = 0;
                static CarMode_t s_prev_mode = CAR_MODE_VOFA;
                uint8_t n_on = 0, i;

                if (CarMode != s_prev_mode)          /* 模式切换沿: 按任务初始化 */
                {
                    s_prev_mode = CarMode;
                    if (CarMode == CAR_MODE_TRACK && TASK_IS_CAR(TaskID))
                    {
                        /* 小车类任务(2/4/5/6)可进TRACK(PA9/VOFA只屏蔽任务3),
                         * 此处按当前任务直接映射串口任务号(枚举顺序即2/3/4/5/6) */
                        static const uint8_t s_task_num_map[5] = { 2, 3, 4, 5, 6 };
                        home_armed = 0;
                        s_arm_cnt = 0;
                        /* MSP->STM32: 发车时刻通知平衡系统
                         * (任务4/5稳O点; 任务6稳在Task6TargetCm自定义点) */
                        Link_NotifyTaskStart(s_task_num_map[TaskID]);
                        if (TaskID == TASK2_LAP)
                        {
                            TrackSpeed = TASK2_TRACK_SPEED;
                            lap_start_ms = 0;   /* 等待重新武装, 防止显示上一圈残留起点 */
                        }
                        else   /* TASK4_AB/TASK5_TRI/TASK6_POS: 进入TRACK立即计时, 不做回A检测 */
                        {
                            TrackSpeed = 0.0f;   /* 由速度剖面逐拍给定, 见下方 ramp 逻辑 */
                            lap_start_ms = g_ms_tick;
                            task_timed_started = 1;
                            if (TaskID == TASK4_AB)
                            {
                                printf("[TASK4] start, ramp %ums -> %.0f, stop after %u ms\n",
                                       (unsigned)TASK4_RAMP_MS, (double)TASK4_TRACK_SPEED,
                                       (unsigned)TASK4_RUN_MS);
                            }
                            else   /* TASK5/TASK6: 同一剖面(4s匀加速0->18+巡航, 30s停) */
                            {
                                if (TaskID == TASK6_POS)
                                    printf("[TASK6] start, ramp %ums -> %.0f, cruise, ball@%.1fcm, stop after %u ms\n",
                                           (unsigned)TASK5_RAMP_MS, (double)TASK5_TRACK_SPEED,
                                           (double)Task6TargetCm, (unsigned)TASK5_RUN_MS);
                                else
                                    printf("[TASK5] start, ramp %ums -> %.0f, cruise, stop after %u ms\n",
                                           (unsigned)TASK5_RAMP_MS, (double)TASK5_TRACK_SPEED,
                                           (unsigned)TASK5_RUN_MS);
                            }
                        }
                    }
                    else
                    {
                        task_timed_started = 0;   /* 退出TRACK(含VOFA手动停车)清除任务四状态 */
                        Link_NotifyTaskStop();    /* MSP->STM32: 停车/回待机时停止平衡 */
                    }
                }

                /* 回A武装仅任务二需要; 任务四不武装(无回A检测) */
                if (CarMode == CAR_MODE_TRACK && TaskID == TASK2_LAP)
                {
                    for (i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++)
                    {
                        if (g_sensor_data[i]) n_on++;
                    }

                    if (!home_armed)
                    {
                        /* 驶离A点标记: 连续3拍 <4路 -> 武装 + 圈速计时起点 */
                        if (n_on < 4)
                        {
                            if (++s_arm_cnt >= 3)
                            {
                                home_armed = 1;
                                lap_start_ms = g_ms_tick;
                                printf("[DBG] ARM at %lu ms, blind until %lu ms\n",
                                       (unsigned long)g_ms_tick,
                                       (unsigned long)(g_ms_tick + HOME_BLIND_MS));
                            }
                        }
                        else
                        {
                            s_arm_cnt = 0;
                        }
                    }
                    else if (g_ms_tick - lap_start_ms == HOME_BLIND_MS)
                    {
                        printf("[DBG] blind off at %lu ms\n", (unsigned long)g_ms_tick);
                    }
                }
            }
#endif /* HOME_DETECT_EN */

            /* 任务四匀加速起步: TrackSpeed 从0线性爬升到 TASK4_TRACK_SPEED(25),
             * 历时 TASK4_RAMP_MS(4s), 之后保持满速直到 8s 定时停车。
             * 每20ms拍速度增量 = 25 * 20 / 4000 = 0.125, 起步冲击加速度大幅减小,
             * 车载小球不会因惯性瞬间后滚偏离O点; 斜坡期间转向限幅随速度同步收紧,
             * 起跑段为直线, 影响可忽略。任务二不受影响(上沿已直接置高速档)。
             * 任务5/任务6匀加速起步+匀速巡航(全程30s): 前 TASK5_RAMP_MS(4s) 匀加速 0->18,
             * 之后匀速 18 巡航直到 30s 定时停车; 无减速段, 起步冲击已柔化,
             * 小球起步不后滚; 停车瞬间的冲击由机械结构吸收(赛题允许)。
             * 任务6与任务5行驶剖面完全相同, 仅钢球稳定目标点不同(q6:Task6TargetCm) */
            if (CarMode == CAR_MODE_TRACK && task_timed_started &&
                (TaskID == TASK4_AB || TaskID == TASK5_TRI || TaskID == TASK6_POS))
            {
                uint32_t ramp_el = g_ms_tick - lap_start_ms;
                if (TaskID == TASK5_TRI || TaskID == TASK6_POS)
                {
                    if (ramp_el < TASK5_RAMP_MS)
                    {
                        /* 前4s: 0 -> 18 匀加速 */
                        TrackSpeed = TASK5_TRACK_SPEED * (float)ramp_el / (float)TASK5_RAMP_MS;
                    }
                    else
                    {
                        /* 之后: 匀速 18 巡航直到 30s 定时停车 */
                        TrackSpeed = TASK5_TRACK_SPEED;
                    }
                }
                else if (ramp_el >= TASK4_RAMP_MS)
                {
                    TrackSpeed = TASK4_TRACK_SPEED;
                }
                else
                {
                    TrackSpeed = TASK4_TRACK_SPEED * (float)ramp_el / (float)TASK4_RAMP_MS;
                }
            }

            if (CarMode == CAR_MODE_TRACK)
            {
                Car_Track_Control();
            }
            else
            {
                Car_SpeedLoop_Control();
                // Car_ExecuteRemoteCommand();  // 开环直驱，测试最大脉冲
            }

            slow_task_cnt++;

            /* 串口 50Hz 输出: 目标L/R, 实际L/R, 线位置, PID输出L/R(最后两列用于观察是否饱和) */
            printf("%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
                   TargetSpeed_L, TargetSpeed_R,
                   s_speed[0], s_speed[1],
                   LinePos,
                   SpeedPID_L.Out, SpeedPID_R.Out);
        }

#if HOME_DETECT_EN
        /* 回A停车判定(主循环高频采样, 仅任务二): 每轮全扫8路(约0.8ms, 比20ms拍密20倍),
         * 高速过A时"4路同亮"的短暂窗口不会再落在两个控制拍之间被漏采。
         * 三个条件复用同一次扫描结果, 满足其一即停:
         *  a) >=4路同亮持续 HOME_N4_HOLD_MS(1ms≈单扫即停) -> 正压A点宽标记(弯道不可能4路同亮);
         *  b) 最右2路(6/7)持续亮满 HOME_R2_HOLD_MS(20ms) -> 压A标记右侧;
         *  c) 右3路(5/6/7)持续亮满 HOME_R3_HOLD_MS(30ms) -> 右斜压A标记。
         * 均在起跑屏蔽期(HOME_BLIND_MS)结束后才启用; 中断即重新计时。 */
        if (TaskID == TASK2_LAP && home_armed && CarMode == CAR_MODE_TRACK &&
            g_ms_tick - lap_start_ms >= HOME_BLIND_MS)   /* 起跑10s内不识别 */
        {
            uint16_t hs[GRAYSCALE_SENSOR_CHANNELS];
            uint8_t  hs_on = 0, hi;

            Grayscale_Sensor_Read_All(hs);
            for (hi = 0; hi < GRAYSCALE_SENSOR_CHANNELS; hi++)
            {
                if (hs[hi]) hs_on++;
            }

            /* a) >=4路同亮持续1ms(≈单扫即停) */
            if (hs_on >= 4)
            {
                if (n4_on_ms == 0)
                {
                    n4_on_ms = g_ms_tick;
                }
                else if (g_ms_tick - n4_on_ms >= HOME_N4_HOLD_MS)
                {
                    Home_Stop(hs_on, 0, 0);
                    n4_on_ms = 0;
                    r2_on_ms = 0;
                    r3_on_ms = 0;
                }
            }
            else
            {
                n4_on_ms = 0;   /* 中断则重新计时 */
            }

            if (CarMode == CAR_MODE_TRACK)   /* 上面可能已停车 */
            {
                /* b) 最右2路(6/7)持续20ms */
                if (hs[6] && hs[7])
                {
                    if (r2_on_ms == 0)
                    {
                        r2_on_ms = g_ms_tick;
                    }
                    else if (g_ms_tick - r2_on_ms >= HOME_R2_HOLD_MS)
                    {
                        Home_Stop(2, 0, 1);
                        r2_on_ms = 0;
                        r3_on_ms = 0;
                        n4_on_ms = 0;
                    }
                }
                else
                {
                    r2_on_ms = 0;   /* 中断则重新计时 */
                }
            }

            if (CarMode == CAR_MODE_TRACK)   /* 上面可能已停车 */
            {
                /* c) 右3路(5/6/7)持续30ms */
                if (hs[5] && hs[6] && hs[7])
                {
                    if (r3_on_ms == 0)
                    {
                        r3_on_ms = g_ms_tick;
                    }
                    else if (g_ms_tick - r3_on_ms >= HOME_R3_HOLD_MS)
                    {
                        Home_Stop(3, 1, 0);
                        r3_on_ms = 0;
                        r2_on_ms = 0;
                        n4_on_ms = 0;
                    }
                }
                else
                {
                    r3_on_ms = 0;   /* 中断则重新计时 */
                }
            }
        }
        else
        {
            n4_on_ms = 0;
            r2_on_ms = 0;
            r3_on_ms = 0;
        }
#endif /* HOME_DETECT_EN */

        /* 任务四(A->B)/任务5/任务6(匀加速+巡航)到时自动停车: 进入TRACK后跑满各自 RUN_MS 即停,
         * 计时起点由模式切换沿设置(task_timed_started=1), 主循环高频检查保证停车时刻准确。
         * 任务5/6无减速段, 30s 时从满速18直接停车 */
        if (task_timed_started && CarMode == CAR_MODE_TRACK &&
            ((TaskID == TASK4_AB && g_ms_tick - lap_start_ms >= TASK4_RUN_MS) ||
             ((TaskID == TASK5_TRI || TaskID == TASK6_POS) &&
              g_ms_tick - lap_start_ms >= TASK5_RUN_MS)))
        {
            TaskTimed_Stop();
        }

        if (slow_task_cnt >= 5)     // 5 * 20ms = 100ms, OLED 保持低频刷新
        {
            slow_task_cnt = 0;

            /* 寻迹中只刷新计时行, 且用突发写(整行约4ms):
             * 普通整屏刷新阻塞约30ms会蒙住高频停车判定(过A窗口仅15~40ms),
             * 4ms远小于15ms窗口, 即使与过A重叠也只会损失一次采样, 不会漏检。
             * 停车回VOFA后恢复刷新并定格显示最终圈速; OLED只显示计时 */
            if (CarMode == CAR_MODE_TRACK)
            {
                if (lap_start_ms != 0)
                {
                    char tbuf[12];
                    uint32_t run_ms = g_ms_tick - lap_start_ms;
                    snprintf(tbuf, sizeof(tbuf), "T:%lu.%lu",
                             (unsigned long)(run_ms / 1000),
                             (unsigned long)((run_ms / 100) % 10));
                    OLED_ShowString16_Fast(0, 5, tbuf);
                }
                continue;
            }

            /* 圈速定格: 回A停车后显示本次成绩(秒.1位小数) */
            {
                OLED_Printf(0, 5, 16, "T:%lu.%lu  ",
                            (unsigned long)(last_lap_ms / 1000),
                            (unsigned long)((last_lap_ms / 100) % 10));
            }
        }
    }
}
