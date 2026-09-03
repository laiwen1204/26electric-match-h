#ifndef __CMD_H
#define __CMD_H

#include <stdint.h>

/* ============================================================================
 * UART1 串口命令接口 (MSP 小车联动 + 人工调试共用)
 * 协议: ASCII 行命令, \r\n 结尾。单命令槽: 主循环未消费时新行丢弃。
 * 主要命令:
 *   q3            要求3 滚球 O->+5->-5 (先停 q4)
 *   q4            要求4/5 稳球在中心 O (先停 q3)
 *   q6:<cm>       要求6 稳球在任意位置 -6~+6cm (先停 q3)
 *   stop          停 q3+q4+电机, 摆杆回平
 *   q3_status / q4_status / cam_status / cam_dump   状态查询
 *   kp/ki/ff/ks/lpf/slew/poskp/posmax...            在线调参
 * ============================================================================ */

uint8_t *cmd_rx_byte_addr(void);   /* HAL_UART_Receive_IT 的目标字节地址 */
void     cmd_rx_byte(uint8_t b);   /* UART1 RX 中断里逐字节喂入 */
void     cmd_process(void);        /* 主循环: 有完整命令行则解析执行 */

#endif /* __CMD_H */
