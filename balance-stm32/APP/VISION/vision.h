#ifndef __VISION_H
#define __VISION_H

#include <stdint.h>

/* ============================================================================
 * 视觉球位反馈链路 (MaixCAM Pro -> UART6)
 * 协议: ASCII 行 "found,pos_cm\r\n", found=1 检测到球 / 0 丢失
 * UART6 RX 中断逐字节喂入环形缓冲, vision_parse_task() 1ms 周期解析,
 * 位置直接采用, 速度由位置差分 + 一阶低通得到。
 * ============================================================================ */

uint8_t *vision_rx_byte_addr(void);      /* HAL_UART_Receive_IT 的目标字节地址 */
void     vision_rx_byte(uint8_t b);      /* UART6 RX 中断里逐字节喂入 */
void     vision_rx_error(uint32_t err);  /* UART6 错误中断里上报错误码 */
void     vision_parse_task(void);        /* 1ms 周期: 消费环形缓冲, 更新球位状态 */

uint8_t  vision_found(void);             /* 最近一帧是否检测到球 */
float    vision_position_cm(void);       /* 球位置 (cm, 摆杆中心 O 为 0) */
float    vision_speed_cm_s(void);        /* 球速度 (cm/s, 一阶低通) */
uint32_t vision_last_sample_ms(void);    /* 最近一次有效位置样本时刻 (HAL_GetTick) */
uint32_t vision_last_rx_ms(void);        /* 最近一次收到合法视觉行时刻 */
void     vision_reset_speed(void);       /* 控制器启动时清零速度估计 */

void     camera_print_status(void);      /* cam_status 命令: 打印链路统计 */
void     camera_dump_recent(void);       /* cam_dump 命令: 打印最近16字节原始数据 */

#endif /* __VISION_H */
