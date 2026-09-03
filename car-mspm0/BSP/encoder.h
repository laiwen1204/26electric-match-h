#ifndef __ENCODER_H
#define __ENCODER_H

#include "ti_msp_dl_config.h"

void Encoder_Init(void);
void Encoder_SampleTick(void);   /* 在 TIMG7 中断中每 20ms 调用一次 */
void Keys_ScanTick(void);        /* 按键1ms扫描消抖(PA9/PA15), 实现在 empty.c */
int16_t Encoder_Get(uint8_t n);

extern volatile uint32_t dbg_int_cnt[4];

#endif
