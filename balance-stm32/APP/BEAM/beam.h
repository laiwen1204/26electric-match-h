#ifndef __BEAM_H
#define __BEAM_H

#include <stdint.h>
#include "m2006.h"

/* ============================================================================
 * 摆杆执行机构应用层 (M2006 + C610 电调, CAN 控制)
 * 拥有: 电机实例 / CAN 看门狗 / 摆杆共享状态(中立点、方向极性、角度限幅、
 * 当前角度指令)。q3(滚球到位) 和 q4(稳球平衡环) 共用这份状态。
 * ============================================================================ */

extern m2006_t beam_motor;           /* M2006 电机实例(位置/速度闭环) */
extern uint8_t beam_output_enabled;  /* 电机输出使能(0=强制零电流) */
extern float   beam_center_deg;      /* 摆杆机械中立点(启动时锁存的电机角) */
extern float   beam_direction;       /* 摆杆方向极性(+1/-1, 安装方向决定) */
extern float   beam_angle_max_deg;   /* 摆杆最大摆角(度) */
extern float   beam_angle_cmd_deg;   /* 当前摆杆角度指令(度) */

void  beam_control_task(void);       /* 1ms: CAN看门狗 + 位置/速度环更新 */
void  beam_print_task(void);         /* 100ms: 打印电机状态(VOFA 波形) */
void  beam_capture_neutral(void);    /* 锁存当前电机角为摆杆中立点 */
void  beam_apply_position(void);     /* 把 beam_angle_cmd_deg 写入位置环目标 */
void  beam_idle(void);               /* 停电机+零电流, 指令角跟随实际角 */
float beam_clamp_angle(float requested_deg);  /* 限幅到 中立点±angle_max */

#endif /* __BEAM_H */
