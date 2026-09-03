#ifndef __USART_H__
#define __USART_H__

#include "ti_msp_dl_config.h"
#include <stdio.h>

/* VOFA 数据包接收 */
extern char vofa_rx_packet[100];
extern volatile uint8_t vofa_rx_flag;

/* UART_LINK RX: STM32 回复行(qX_started/qX_start_rejected), 主循环消费 */
extern char link_rx_line[96];
extern volatile uint8_t link_rx_flag;

extern volatile uint8_t  recv0_buff[128];
extern volatile uint16_t recv0_length;
extern volatile uint8_t  recv0_flag;

void USART_Init(void);
void USART_SendData(unsigned char data);
void USART_SendArray(uint8_t *arr, uint8_t len);

/* UART DMA 发送接口 */
void UART_DMA_SendStart(void);

/* ==================== MSP -> STM32 平衡控制链路 (UART_LINK) ====================
 * SysConfig 中 UART_LINK 实例 = UART3, TX=PB12 -> STM32 USART1 RX(PB7),
 * RX=PB13 <- STM32 USART1 TX(PA9), 共地, 115200 8N1。
 * 未配置 UART_LINK 时以下函数为空操作, 不影响编译。 */
void Link_SendStr(const char *s);
void Link_NotifyTaskStart(uint8_t task_num); /* 2=TASK2 3=TASK3 4=TASK4 5=TASK5 6=TASK6 */
void Link_NotifyTaskStop(void);

#endif
