#ifndef __TIMER_H
#define __TIMER_H

#include "ti_msp_dl_config.h"

extern volatile uint8_t timer_flag;
extern volatile uint32_t g_ms_tick;   /* 1ms 自由运行时基(TIMG7中断累加), 圈速计时用 */

void Timer_Init(void);

#endif
