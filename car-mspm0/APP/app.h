#ifndef __APP_H
#define __APP_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "config.h"
#include "car.h"
#include "encoder.h"
#include "pid.h"
#include "usart.h"
#include "grayscale_sensor.h"

typedef enum {
    CAR_CMD_STOP = 0,
    CAR_CMD_FORWARD,
    CAR_CMD_BACK,
    CAR_CMD_LEFT,
    CAR_CMD_RIGHT
} CarCommand_t;

typedef enum {
    CAR_MODE_VOFA = 0,
    CAR_MODE_TRACK
} CarMode_t;

/* 任务选择: 由第二按键(PA15)在待机(VOFA)状态下短按切换 */
typedef enum {
    TASK2_LAP = 0,  /* 要求2: 高速跑整圈, 回A点自动停车并计时 */
    TASK3_ROLL,     /* 要求3: 纯STM32滚球到位, 小车不动, 选中即由串口发 q3 触发 */
    TASK4_AB,       /* 要求4: A->B 匀加速起步+低速行驶, 到时自动停车并计时 */
    TASK5_TRI,      /* 要求5: 匀加速起步30s, 前4s匀加速0->18, 之后匀速巡航, 到时定时停车 */
    TASK6_POS       /* 要求6: 纯STM32任意点停球, 小车不动, 选中即发 q6:Task6TargetCm */
} TaskID_t;

/* 小车类任务(需要发车/进TRACK): 2/4/5/6; 任务3是纯STM32滚球动作, 小车不动 */
#define TASK_IS_CAR(t)   ((t) != TASK3_ROLL)

extern CarCommand_t CarCommand;
extern CarMode_t CarMode;
extern TaskID_t TaskID;
extern uint16_t BaseSpeed;
extern float TrackSpeed;
extern PID_t PosPID;
extern float s_speed[2];
extern float TargetSpeed_L;
extern float TargetSpeed_R;
extern float LinePos;
extern uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS]; /* 8路数字位(1=看到黑线, OLED显示用) */
extern float Task6TargetCm;   /* 要求6: q6 停球目标位置(cm, -6~+6, 杆中心为0) */

extern PID_t SpeedPID_L;
extern PID_t SpeedPID_R;

void Encoder_UpdateAll(void);
void Car_ExecuteRemoteCommand(void);
void SpeedPID_Init(void);
void PosPID_Init(void);
void Car_SpeedLoop_Control(void);
void Car_Track_Control(void);
float CalcLinePosition(const uint16_t *bits);   /* 纯数字量质心, 全丢返回-1 */

#endif
