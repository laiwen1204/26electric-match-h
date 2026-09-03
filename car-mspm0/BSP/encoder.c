#include "encoder.h"

/*
 * GPIO软件正交解码 (Plan B)
 *
 * MSPM0G3507只有TIMG8支持硬件QEI，TIMG0/TIMG6不支持。
 * 改用GPIO双边沿中断 + 4倍频状态机实现软件计数。
 *
 * 引脚分配（由syscfg GPIO_ENCODER实例初始化）:
 *   左轮: PB0 (A相), PB1 (B相)  → GPIOB
 *   右轮: PB10 (A相), PB11 (B相) → GPIOB
 *   所有GPIO中断共享 GROUP1 中断线
 *
 * 4倍频: AB两通道的所有上升/下降沿都计数，13PPR×30:1×4=1560 counts/rev
 *
 * 采样方式（重要）:
 *   速度 = 每20ms的脉冲增量。采样必须在 TIMG7 定时中断里做
 *   （Encoder_SampleTick），保证20ms窗口严格准确。
 *   若放在主循环采样，OLED软件I2C刷新等长耗时操作会把窗口拉长到
 *   40/60/80ms，速度读数出现 2x/3x/4x 假尖峰（并非编码器噪声）。
 */

static volatile int32_t enc_count[2] = {0, 0};
static uint8_t prev_state[2] = {0, 0};

/* 最近一个20ms窗口的脉冲增量（由 Encoder_SampleTick 更新） */
static volatile int16_t s_delta[2] = {0, 0};

/* 调试：每通道中断触发计数 */
volatile uint32_t dbg_int_cnt[4] = {0, 0, 0, 0};

/* 4倍频正交解码查找表: index=(old<<2)|new, A=bit1, B=bit0 */
static const int8_t quad_table[16] = {
     0, +1, -1,  0,
    -1,  0,  0, +1,
    +1,  0,  0, -1,
     0, -1, +1,  0,
};

void Encoder_Init(void)
{
    /*
     * 引脚配置已由 SYSCFG_DL_GPIO_init() 完成（syscfg GPIO_ENCODER实例:
     * INPUT + PULL_UP + RISE_FALL + clearInterruptStatus + enableInterrupt），
     * 此处只需读取初始AB状态然后使能NVIC。
     */
    uint32_t rl = DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_0 | DL_GPIO_PIN_1);
    uint32_t rr = DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_10 | DL_GPIO_PIN_11);
    prev_state[0] = (uint8_t)(((rl & DL_GPIO_PIN_1) ? 2U : 0U) | ((rl & DL_GPIO_PIN_0) ? 1U : 0U));
    prev_state[1] = (uint8_t)(((rr & DL_GPIO_PIN_11) ? 2U : 0U) | ((rr & DL_GPIO_PIN_10) ? 1U : 0U));

    /* GPIOB_INT_IRQn 由 main() 使能 */
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1))
    {
        case DL_INTERRUPT_GROUP1_IIDX_GPIOB:
        {
            /* 左轮编码器 (GPIOB PB0=A相, PB1=B相) */
            uint32_t stl = DL_GPIO_getEnabledInterruptStatus(GPIOB,
                DL_GPIO_PIN_0 | DL_GPIO_PIN_1);
            if (stl) {
                if (stl & DL_GPIO_PIN_0) dbg_int_cnt[0]++;
                if (stl & DL_GPIO_PIN_1) dbg_int_cnt[1]++;
                DL_GPIO_clearInterruptStatus(GPIOB, stl);
                uint32_t raw = DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_0 | DL_GPIO_PIN_1);
                uint8_t ns = (uint8_t)(((raw & DL_GPIO_PIN_1) ? 2U : 0U) | ((raw & DL_GPIO_PIN_0) ? 1U : 0U));
                uint8_t idx = (uint8_t)((prev_state[0] << 2) | ns);
                enc_count[0] += quad_table[idx];
                prev_state[0] = ns;
            }

            /* 右轮编码器 (GPIOB PB10=A相, PB11=B相) */
            uint32_t str_ = DL_GPIO_getEnabledInterruptStatus(GPIOB,
                DL_GPIO_PIN_10 | DL_GPIO_PIN_11);
            if (str_) {
                if (str_ & DL_GPIO_PIN_10) dbg_int_cnt[2]++;
                if (str_ & DL_GPIO_PIN_11) dbg_int_cnt[3]++;
                DL_GPIO_clearInterruptStatus(GPIOB, str_);
                uint32_t raw = DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_10 | DL_GPIO_PIN_11);
                uint8_t ns = (uint8_t)(((raw & DL_GPIO_PIN_11) ? 2U : 0U) | ((raw & DL_GPIO_PIN_10) ? 1U : 0U));
                uint8_t idx = (uint8_t)((prev_state[1] << 2) | ns);
                enc_count[1] += quad_table[idx];
                prev_state[1] = ns;
            }
            break;
        }

        default:
            break;
    }
}

/* 在 TIMG7 1ms 中断里每 20ms 调用一次：锁定精确的采样窗口 */
void Encoder_SampleTick(void)
{
    static int32_t s_last[2] = {0, 0};
    int i;

    for (i = 0; i < 2; i++)
    {
        int32_t now = enc_count[i];
        int32_t delta = now - s_last[i];
        s_last[i] = now;

        if (delta > 32767) delta = 32767;
        else if (delta < -32768) delta = -32768;
        s_delta[i] = (int16_t)delta;
    }
}

/* 返回最近一个 20ms 窗口的脉冲增量（语义与原来一致） */
int16_t Encoder_Get(uint8_t n)
{
    if (n < 1 || n > 2) return 0;
    return s_delta[n - 1];
}
