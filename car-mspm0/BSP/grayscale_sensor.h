#ifndef __GRAYSCALE_SENSOR_H
#define __GRAYSCALE_SENSOR_H

#include <stdint.h>
#include "ti_msp_dl_config.h"
#include "delay.h"

#define SENSOR_AD0_PORT         GrayS_PORT
#define SENSOR_AD0_PIN          GrayS_PIN_0_PIN

#define SENSOR_AD1_PORT         GrayS_PORT
#define SENSOR_AD1_PIN          GrayS_PIN_1_PIN

#define SENSOR_AD2_PORT         GrayS_PORT
#define SENSOR_AD2_PIN          GrayS_PIN_2_PIN

#define GrayS_OUT_PORT          GrayS_PORT
#define GrayS_OUT_PIN           GrayS_PIN_3_PIN

#define GRAYSCALE_PIN_WRITE(port, pin, state) do { \
    if(state) DL_GPIO_setPins(port, pin); \
    else DL_GPIO_clearPins(port, pin); \
} while(0)

#define SENSOR_AD0_WRITE(state)  GRAYSCALE_PIN_WRITE(SENSOR_AD0_PORT, SENSOR_AD0_PIN, state)
#define SENSOR_AD1_WRITE(state)  GRAYSCALE_PIN_WRITE(SENSOR_AD1_PORT, SENSOR_AD1_PIN, state)
#define SENSOR_AD2_WRITE(state)  GRAYSCALE_PIN_WRITE(SENSOR_AD2_PORT, SENSOR_AD2_PIN, state)

#define SENSOR_OUT_READ()        (!!(DL_GPIO_readPins(GrayS_OUT_PORT, GrayS_OUT_PIN)))

#define GRAYSCALE_SENSOR_CHANNELS   8

/* 赛道极性: 黑线白底 -> 探头照到黑线时对应位 = 1。
 * 该灰度模块通常压黑线时指示灯亮、OUT=1, 与上面一致;
 * 若装车后发现全反(白底显示1), 改为 0 即可。 */
#define GRAYSCALE_ON_BLACK   1

/* 模块左右镜像: 若物理第0路位于车右(模块装反),
 * 置 1 将通道顺序反转, 使 out[0] 始终对应车左。
 * 判断方法: 黑线放车最左侧探头下, OLED S: 最左字符应为 1。 */
#define GRAYSCALE_MIRROR     0

void Grayscale_Sensor_Init(void);
void Grayscale_Sensor_Read_All(uint16_t* sensor_values);
uint16_t Grayscale_Sensor_Read_Single(uint8_t channel);   /* 读单路(逻辑通道号, 已含极性/镜像), 1=看到黑线 */

#endif
