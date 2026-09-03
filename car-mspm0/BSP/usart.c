#include "usart.h"

#define RE_0_BUFF_LEN_MAX   128

volatile uint8_t  recv0_buff[RE_0_BUFF_LEN_MAX] = {0};
volatile uint16_t recv0_length = 0;
volatile uint8_t  recv0_flag = 0;

/* VOFA 数据包接收 */
char vofa_rx_packet[100];
volatile uint8_t vofa_rx_flag = 0;

/* ---- UART_LINK(UART3) RX: 接收 STM32 回复(q4_started/q4_start_rejected等) ----
 * 行缓冲+标志, 主循环 Link 处理在 empty.c, 用于确认任务指令是否被STM32执行 */
char link_rx_line[96];
volatile uint8_t link_rx_flag = 0;
static volatile uint8_t s_link_rx_index = 0;

/* ============================================================================
 * UART TX 方式开关：0=轮询（与参考工程一致，稳定）；1=DMA 环形缓冲后台发送
 * ============================================================================ */
#define UART_TX_USE_DMA    1

#if UART_TX_USE_DMA
/* ============================================================================
 * UART DMA 发送环形缓冲区
 * ============================================================================ */
#define UART_TX_BUF_SIZE    512

static uint8_t  uart_tx_buf[UART_TX_BUF_SIZE];
static volatile uint16_t uart_tx_head = 0;
static volatile uint16_t uart_tx_tail = 0;
static volatile uint8_t  uart_tx_dma_busy = 0;

/* 本次 DMA 实际发送的字节数，用于 ISR 中更新 tail */
static volatile uint16_t uart_tx_dma_len = 0;
#endif /* UART_TX_USE_DMA */

void USART_Init(void)
{
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

#ifdef UART_LINK_INST
    /* UART_LINK RX 中断: 接收 STM32 回复行(qX_started/qX_start_rejected),
     * 主循环在 empty.c 消费 link_rx_flag, OLED显示+VOFA回显 */
    DL_UART_clearInterruptStatus(UART_LINK_INST, DL_UART_INTERRUPT_RX);
    DL_UART_enableInterrupt(UART_LINK_INST, DL_UART_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_LINK_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_LINK_INST_INT_IRQN);
#endif
}

void USART_SendData(unsigned char data)
{
    /* 保持向后兼容：若 DMA 未就绪仍可用轮询（初始化阶段） */
    while (DL_UART_isBusy(UART_0_INST) == true);
    DL_UART_Main_transmitData(UART_0_INST, data);
}

void USART_SendArray(uint8_t *arr, uint8_t len)
{
    uint8_t i = 0;
    for (i = 0; i < len; i++)
    {
        USART_SendData(arr[i]);
    }
}

#if !defined(__MICROLIB)
#if (__ARMCLIB_VERSION <= 6000000)
struct __FILE
{
    int handle;
};
#endif

FILE __stdout;

void _sys_exit(int x)
{
    x = x;
}
#endif

/* ============================================================================
 * fputc — 轮询发送（UART_TX_USE_DMA=1 时走 DMA 环形缓冲后台发送）
 * ============================================================================ */
int fputc(int ch, FILE *stream)
{
#if UART_TX_USE_DMA
    uint16_t next_head;

    (void)stream;

    next_head = (uart_tx_head + 1) % UART_TX_BUF_SIZE;

    /* 缓冲区满时等待（一般不会满，因为 DMA 发送很快） */
    while (next_head == uart_tx_tail) {
        ; /* 简单自旋等待，已比 UART 轮询发送轻量得多 */
    }

    uart_tx_buf[uart_tx_head] = (uint8_t)ch;
    uart_tx_head = next_head;

    /* 如果 DMA 空闲，立即启动发送 */
    if (!uart_tx_dma_busy) {
        UART_DMA_SendStart();
    }
#else
    /* 轮询发送（与参考工程一致） */
    (void)stream;
    while (DL_UART_isBusy(UART_0_INST) == true);
    DL_UART_Main_transmitData(UART_0_INST, (uint8_t)ch);
#endif

    return ch;
}

int fputs(const char* restrict s, FILE* restrict stream) {
    uint16_t char_len = 0;
    (void)stream;
    while (*s != 0)
    {
        fputc(*s++, NULL);
        char_len++;
    }
    return char_len;
}

int puts(const char* _ptr)
{
    return fputs(_ptr, stdout);
}

#if UART_TX_USE_DMA
/* ============================================================================
 * UART DMA 发送启动 / 续传
 * 从 uart_tx_tail 开始发送一段连续数据到缓冲区末尾或 head 位置
 * ============================================================================ */
void UART_DMA_SendStart(void)
{
    uint16_t len;

    /* 关闭中断保护临界区，防止与 DMA ISR 竞争 */
    __disable_irq();

    if (uart_tx_head == uart_tx_tail) {
        uart_tx_dma_busy = 0;
        __enable_irq();
        return;
    }

    if (uart_tx_head > uart_tx_tail) {
        len = uart_tx_head - uart_tx_tail;
    } else {
        /* 回绕：先发送到缓冲区末尾 */
        len = UART_TX_BUF_SIZE - uart_tx_tail;
    }

    uart_tx_dma_busy = 1;
    uart_tx_dma_len  = len;

    /* ---- 配置 UART TX DMA ---- */
    DL_DMA_disableChannel(DMA, DMA_UART0_TX_CHAN_ID);

    DL_DMA_setSrcAddr(DMA, DMA_UART0_TX_CHAN_ID,
                      (uint32_t)&uart_tx_buf[uart_tx_tail]);
    DL_DMA_setDestAddr(DMA, DMA_UART0_TX_CHAN_ID,
                       (uint32_t)(&UART_0_INST->TXDATA));
    DL_DMA_setTransferSize(DMA, DMA_UART0_TX_CHAN_ID, len);

    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL2);
    DL_DMA_enableInterrupt(DMA, DL_DMA_INTERRUPT_CHANNEL2);
    DL_DMA_enableChannel(DMA, DMA_UART0_TX_CHAN_ID);

    __enable_irq();
}

/* ============================================================================
 * DMA 全局中断 — 目前仅处理 UART0 TX 完成事件
 * ============================================================================ */
void DMA_IRQHandler(void)
{
    uint32_t pending = DL_DMA_getPendingInterrupt(DMA);

    /* 处理 UART0 TX DMA 完成 */
    if (pending == DL_DMA_EVENT_IIDX_DMACH2) {
        DL_DMA_disableChannel(DMA, DMA_UART0_TX_CHAN_ID);
        DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL2);

        /* 更新 tail */
        uart_tx_tail = (uart_tx_tail + uart_tx_dma_len) % UART_TX_BUF_SIZE;

        /* 如果还有数据，继续发送 */
        if (uart_tx_head != uart_tx_tail) {
            UART_DMA_SendStart();
        } else {
            uart_tx_dma_busy = 0;
        }
    }
    /* 如后续需要扩展 SPI DMA 中断，可在此添加 CH0/CH1 处理 */
}
#endif /* UART_TX_USE_DMA */

/* ============================================================================
 * UART_0 中断服务函数：接收（保持原有逻辑不变）
 * ============================================================================ */
void UART_0_INST_IRQHandler(void)
{
    static uint8_t vofa_rx_index = 0;
    uint8_t receivedData = 0;

    switch (DL_UART_getPendingInterrupt(UART_0_INST))
    {
        case DL_UART_IIDX_RX:
            receivedData = DL_UART_Main_receiveData(UART_0_INST);

            /* VOFA行分隔协议：以 \r 或 \n 结尾 */
            if (receivedData == '\n' || receivedData == '\r')
            {
                if (vofa_rx_index > 0 && vofa_rx_flag == 0)
                {
                    vofa_rx_packet[vofa_rx_index] = '\0';
                    vofa_rx_flag = 1;
                    vofa_rx_index = 0;
                }
            }
            else
            {
                if (vofa_rx_index < 99 && vofa_rx_flag == 0)
                {
                    vofa_rx_packet[vofa_rx_index++] = (char)receivedData;
                }
            }

            /* 同时保留原始字节缓冲（兼容旧代码） */
            if (recv0_length < RE_0_BUFF_LEN_MAX - 1)
            {
                recv0_buff[recv0_length++] = receivedData;
            }
            else
            {
                recv0_length = 0;
            }
            recv0_flag = 1;
            break;

        default:
            break;
    }
}

/* UART_LINK(UART3) RX 中断: 行缓冲, \r或\n结尾置标志 */
void UART_LINK_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_LINK_INST))
    {
        case DL_UART_IIDX_RX:
        {
            uint8_t b = DL_UART_Main_receiveData(UART_LINK_INST);
            if (b == '\r' || b == '\n')
            {
                if (s_link_rx_index > 0 && link_rx_flag == 0)
                {
                    link_rx_line[s_link_rx_index] = '\0';
                    link_rx_flag = 1;
                    s_link_rx_index = 0;
                }
                else
                {
                    s_link_rx_index = 0;   /* 主循环没消费完就丢弃(只留最新) */
                }
            }
            else if (s_link_rx_index < sizeof(link_rx_line) - 1)
            {
                link_rx_line[s_link_rx_index++] = (char)b;
            }
            break;
        }
        default:
            break;
    }
}

/* ==================== MSP -> STM32 平衡控制链路 (UART_LINK/UART3) ============
 * 协议: ASCII 行命令, \r\n 结尾, 直接复用 STM32 现有串口命令表:
 *   "q4\r\n"      -> STM32 停q3并启动钢球稳中心O (任务4/5发车时发)
 *   "q3\r\n"      -> STM32 停q4并启动滚球到位状态机 (任务3, 选中即发)
 *   "q6:x.x\r\n"  -> STM32 停q3并任意点停球, x=目标cm(-6~+6) (任务6, 选中即发)
 *   "stop\r\n"    -> STM32 停q3+q4+电机, 摆杆回平 (任务2发车/任何停车时发)
 * 重要: 每次只发【一条】命令! STM32 USART1 是单命令槽(m2006_cmd_ready),
 * 主循环未消费时新行会被丢弃; 连发多条会因STM32主循环被printf/视觉占住
 * 而随机丢命令(曾表现为切到任务4/5有时不进平衡)。互斥(q3/q4/q6)已由
 * STM32命令处理器内部先停后启, 无需MSP连发stop。 */
void Link_SendStr(const char *s)
{
#ifdef UART_LINK_INST
    while (*s)
    {
        DL_UART_transmitDataBlocking(UART_LINK_INST, (uint8_t)*s++);
    }
#else
    (void)s;   /* SysConfig 未配置 UART_LINK: 空操作 */
#endif
}

/* 要求6默认停球目标位置, 定义在 APP/app.c, 可用VOFA slider在线修改 */
extern float Task6TargetCm;

void Link_NotifyTaskStart(uint8_t task_num)
{
#ifdef UART_LINK_INST
    char buf[16];
    switch (task_num)
    {
        case 3:
            Link_SendStr("q3\r\n");      /* 任务3: 滚球到位(STM32内先停q4) */
            break;
        case 6:
            snprintf(buf, sizeof(buf), "q6:%.1f\r\n", (double)Task6TargetCm);
            Link_SendStr(buf);           /* 任务6: 任意点停球(STM32内先停q3) */
            break;
        case 4:
        case 5:
            Link_SendStr("q4\r\n");      /* 任务4/5: 稳中心O(STM32内先停q3) */
            break;
        default:
            Link_SendStr("stop\r\n");    /* 任务2: 平衡/滚球全停 */
            break;
    }
#else
    (void)task_num;
#endif
}

void Link_NotifyTaskStop(void)
{
    /* 停车/回待机: stop 一条命令停 q3+q4+电机, 摆杆回平 */
    Link_SendStr("stop\r\n");
}
