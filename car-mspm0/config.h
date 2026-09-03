#ifndef __CONFIG_H
#define __CONFIG_H

/* MG310霍尔编码电机参数：编码器13线，减速比30，4倍频 */
#define ENCODER_LINES           13
#define REDUCTION_RATIO         30
#define SAMPLING_TIME_MS        20
#define PULSES_PER_REVOLUTION   (ENCODER_LINES * REDUCTION_RATIO * 4)
#define LPF_ALPHA               0.25f
#define ANGLE_PID_MAX           400
#define ANGLE_PID_MIN           -400
#define BASE_SPEED_DEFAULT      12

/* 任务配置: TASK2=要求2(高速跑整圈回A停车计时), TASK4=要求4(A->B低速行驶, 到时自停) */
#define TASK2_TRACK_SPEED       29.0f
#define TASK4_TRACK_SPEED       25.0f
#define TASK4_RUN_MS            8000   /* 任务四行驶时长, 到时自动停车 */
#define TASK4_RAMP_MS           4000   /* 任务四匀加速起步时长: 0->TASK4_TRACK_SPEED 线性爬坡,
                                        * 减小起步冲击加速度, 防车载小球因惯性后滚偏离O点 */

/* 任务5(要求5): 匀加速起步+匀速巡航, 全程30s
 * 前 TASK5_RAMP_MS 匀加速 0->TASK5_TRACK_SPEED, 之后匀速巡航;
 * 无减速段, 到 TASK5_RUN_MS 由定时停车直接停止 */
#define TASK5_TRACK_SPEED       18.0f
#define TASK5_RUN_MS            30000
#define TASK5_RAMP_MS           4000

#endif
