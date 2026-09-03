#ifndef __Q4_H
#define __Q4_H

#include <stdint.h>

/* ============================================================================
 * 要求4/5: 小车行驶中钢球稳定在摆杆中心 O 附近(误差 <=1cm)
 * 要求6:   同一平衡环, 目标改为任意指定位置(q6 命令)
 * 双区增益调度 PD + 破粘滞全幅脉冲 + 脉冲后主动阻尼冷却;
 * 视觉短时丢帧保持摆杆水平, 数据恢复后自动继续平衡(行驶震动下丢帧是常态)。
 * ============================================================================ */

void    q4_control_task(void);   /* 10ms 周期 */
uint8_t q4_start(void);          /* 启动平衡(与 q3 互斥), 1=成功/已在运行 0=拒绝 */
void    q4_stop(void);           /* 停止并切断电机输出 */
uint8_t q4_is_idle(void);        /* 1=空闲(未运行) */
void    q4_print_status(void);   /* q4_status 命令: 打印平衡环内部量 */

void  q4_set_target(float cm);   /* 设定稳球目标位置(-6~+6cm, q6/q4target 用) */
float q4_get_target(void);
void  q4_set_kp(float kp);       /* 在线调参(q4kp/q4kd/q4max 命令) */
void  q4_set_kd(float kd);
float q4_set_effort_max(float deg);  /* 返回限幅后的实际生效值 */

#endif /* __Q4_H */
