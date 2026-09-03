#include "grayscale_sensor.h"

static void _delay_us(volatile uint32_t us)
{
    delay_us(us);
}

static void _select_channel(uint8_t channel)
{
    SENSOR_AD0_WRITE((channel >> 0) & 0x01);
    SENSOR_AD1_WRITE((channel >> 1) & 0x01);
    SENSOR_AD2_WRITE((channel >> 2) & 0x01);
}

static uint16_t Read_OUT_value(void)
{
    return SENSOR_OUT_READ();
}

void Grayscale_Sensor_Init(void)
{
}

void Grayscale_Sensor_Read_All(uint16_t* sensor_values)
{
    uint8_t i;
    for (i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++)
    {
        uint16_t v;
        _select_channel(i);
        _delay_us(100);
        v = Read_OUT_value();
#if !GRAYSCALE_ON_BLACK
        v = !v;                 /* 极性反转: 统一成 1=看到黑线 */
#endif
#if GRAYSCALE_MIRROR
        sensor_values[GRAYSCALE_SENSOR_CHANNELS - 1 - i] = v;
#else
        sensor_values[i] = v;
#endif
    }
}

//读取单路(逻辑通道号, 含极性/镜像处理), 返回 1=看到黑线
uint16_t Grayscale_Sensor_Read_Single(uint8_t channel)
{
    uint16_t v;
    if (channel >= GRAYSCALE_SENSOR_CHANNELS) return 0;
#if GRAYSCALE_MIRROR
    channel = GRAYSCALE_SENSOR_CHANNELS - 1 - channel;
#endif
    _select_channel(channel);
    _delay_us(50);          /* 主循环高频调用, 稳定延时取短一些 */
    v = Read_OUT_value();
#if !GRAYSCALE_ON_BLACK
    v = !v;
#endif
    return v;
}
