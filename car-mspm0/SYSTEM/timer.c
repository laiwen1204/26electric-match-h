#include "timer.h"
#include "encoder.h"

volatile uint8_t timer_flag = 0;
volatile uint32_t g_ms_tick = 0;   /* 1ms 自由运行时基 */

/**
 * TIMG7 系统时基已由 SysConfig 初始化
 * (BUSCLK 32MHz / 32 预分频 = 1MHz, 周期 1000, ZERO 中断)
 * 注意: SysConfig 生成的初始化不会启动计数器, 必须在这里手动启动,
 * 否则 ZERO 中断永远不触发, timer_flag 不置位, 主循环假死!
 */
void Timer_Init(void)
{
    DL_TimerG_startCounter(SysTick_Timer_INST);
    NVIC_EnableIRQ(SysTick_Timer_INST_INT_IRQN);
}

/* TIMG7 零事件中断，每1ms触发一次 */
void SysTick_Timer_INST_IRQHandler(void)
{
    static uint8_t ms_cnt = 0;

    switch (DL_TimerG_getPendingInterrupt(SysTick_Timer_INST))
    {
        case DL_TIMER_IIDX_ZERO:
            DL_Timer_clearInterruptStatus(SysTick_Timer_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
            g_ms_tick++;
            /* 按键1ms扫描消抖(PA9/PA15), 事件标志由主循环消费 */
            Keys_ScanTick();
            ms_cnt++;
            if (ms_cnt >= 20)
            {
                ms_cnt = 0;
                /* 在中断里采样编码器: 保证 20ms 窗口严格准确,
                   避免主循环被 OLED 等长耗时操作拉长窗口产生速度假尖峰 */
                Encoder_SampleTick();
                timer_flag = 1;
            }
            break;
        default:
            break;
    }
}
