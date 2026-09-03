#include "oled_software_i2c.h"
#include "oledfont.h"
#include "delay.h"
#include <stdarg.h>
#include <stdio.h>

// OLED显存
// [0]0 1 2 3 ... 127
// [1]0 1 2 3 ... 127
// [2]0 1 2 3 ... 127
// [3]0 1 2 3 ... 127
// [4]0 1 2 3 ... 127
// [5]0 1 2 3 ... 127
// [6]0 1 2 3 ... 127
// [7]0 1 2 3 ... 127

void OLED_ColorTurn(uint8_t i)
{
    if (i == 0)
        OLED_WR_Byte(0xA6, OLED_CMD);
    if (i == 1)
        OLED_WR_Byte(0xA7, OLED_CMD);
}

void OLED_DisplayTurn(uint8_t i)
{
    if (i == 0) {
        OLED_WR_Byte(0xC8, OLED_CMD);
        OLED_WR_Byte(0xA1, OLED_CMD);
    }
    if (i == 1) {
        OLED_WR_Byte(0xC0, OLED_CMD);
        OLED_WR_Byte(0xA0, OLED_CMD);
    }
}

static void I2C_Start(void)
{
    OLED_SDA_Set();
    delay_us(2);
    OLED_SCL_Set();
    delay_us(2);
    OLED_SDA_Clr();
    delay_us(2);
    OLED_SCL_Clr();
}

static void I2C_Stop(void)
{
    OLED_SDA_Clr();
    OLED_SCL_Set();
    OLED_SDA_Set();
}

static void I2C_WaitAck(void)
{
    OLED_SDA_Set();
    OLED_SCL_Set();
    OLED_SCL_Clr();
}

static void Send_Byte(uint8_t dat)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        OLED_SCL_Clr();
        if (dat & 0x80)
            OLED_SDA_Set();
        else
            OLED_SDA_Clr();
        OLED_SCL_Set();
        delay_us(2);
        OLED_SCL_Clr();
        dat <<= 1;
    }
}

void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    I2C_Start();
    Send_Byte(0x78);
    I2C_WaitAck();
    if (mode)
        Send_Byte(0x40);
    else
        Send_Byte(0x00);
    I2C_WaitAck();
    Send_Byte(dat);
    I2C_WaitAck();
    I2C_Stop();
}

void OLED_Set_Pos(uint8_t x, uint8_t y)
{
    OLED_WR_Byte(0xb0 + y, OLED_CMD);
    OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
    OLED_WR_Byte((x & 0x0f), OLED_CMD);
}

void OLED_Display_On(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD);
    OLED_WR_Byte(0X14, OLED_CMD);
    OLED_WR_Byte(0XAF, OLED_CMD);
}

void OLED_Display_Off(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD);
    OLED_WR_Byte(0X10, OLED_CMD);
    OLED_WR_Byte(0XAE, OLED_CMD);
}

void OLED_Clear(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++)
    {
        OLED_WR_Byte(0xb0 + i, OLED_CMD);
        OLED_WR_Byte(0x00, OLED_CMD);
        OLED_WR_Byte(0x10, OLED_CMD);
        for (n = 0; n < 128; n++)
            OLED_WR_Byte(0, OLED_DATA);
    }
}

void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t sizey)
{
    uint8_t c = 0, sizex = sizey / 2;
    uint16_t i = 0, size1;
    if (sizey == 8)
        size1 = 6;
    else
        size1 = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * (sizey / 2);
    c = chr - ' ';
    OLED_Set_Pos(x, y);
    for (i = 0; i < size1; i++)
    {
        if (i % sizex == 0 && sizey != 8)
            OLED_Set_Pos(x, y++);
        if (sizey == 8)
            OLED_WR_Byte(asc2_0806[c][i], OLED_DATA);
        else if (sizey == 16)
            OLED_WR_Byte(asc2_1608[c][i], OLED_DATA);
        else
            return;
    }
}

uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--)
        result *= m;
    return result;
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t sizey)
{
    uint8_t t, temp, m = 0;
    uint8_t enshow = 0;
    if (sizey == 8)
        m = 2;
    for (t = 0; t < len; t++)
    {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                OLED_ShowChar(x + (sizey / 2 + m) * t, y, ' ', sizey);
                continue;
            }
            else
                enshow = 1;
        }
        OLED_ShowChar(x + (sizey / 2 + m) * t, y, temp + '0', sizey);
    }
}

void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t sizey)
{
    uint8_t j = 0;
    while (chr[j] != '\0')
    {
        OLED_ShowChar(x, y, chr[j++], sizey);
        if (sizey == 8)
            x += 6;
        else
            x += sizey / 2;
    }
}

/* 连续突发写一行16px字符串(仅支持sizey=16):
 * 普通ShowChar每个字节都是一次完整I2C事务(约100us), 8字符计时行要~17ms,
 * 寻迹中会阻塞高频停车判定造成漏检; 这里定位后START一次连续发数据,
 * 整行耗时压到约4ms (< 过A标记最小窗口15ms, 安全)。
 * 原理: 16px字符高2页, 上页8字节+下页8字节, 分两页各突发一次。 */
void OLED_ShowString16_Fast(uint8_t x, uint8_t y, const char *chr)
{
    uint8_t page, j, k, c;
    for (page = 0; page < 2; page++)
    {
        OLED_Set_Pos(x, y + page);
        I2C_Start();
        Send_Byte(0x78);        /* SSD1306 I2C地址 */
        I2C_WaitAck();
        Send_Byte(0x40);        /* Co=0, D/C#=1: 后续全是显存数据 */
        I2C_WaitAck();
        for (j = 0; chr[j] != '\0'; j++)
        {
            c = chr[j] - ' ';
            for (k = 0; k < 8; k++)     /* 每字符每页8列 */
            {
                Send_Byte(asc2_1608[c][page * 8 + k]);
                I2C_WaitAck();
            }
        }
        I2C_Stop();
    }
}

void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t no, uint8_t sizey)
{
    uint16_t i, size1 = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
    for (i = 0; i < size1; i++)
    {
        if (i % sizey == 0)
            OLED_Set_Pos(x, y++);
        if (sizey == 16)
            OLED_WR_Byte(Hzk[no][i], OLED_DATA);
        else
            return;
    }
}

void OLED_DrawBMP(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[])
{
    uint16_t j = 0;
    uint8_t i, m;
    sizey = sizey / 8 + ((sizey % 8) ? 1 : 0);
    for (i = 0; i < sizey; i++)
    {
        OLED_Set_Pos(x, i + y);
        for (m = 0; m < sizex; m++)
        {
            OLED_WR_Byte(BMP[j++], OLED_DATA);
        }
    }
}

void OLED_Init(void)
{
    delay_ms(200);

    OLED_WR_Byte(0xAE, OLED_CMD);
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0x10, OLED_CMD);
    OLED_WR_Byte(0x40, OLED_CMD);
    OLED_WR_Byte(0x81, OLED_CMD);
    OLED_WR_Byte(0xCF, OLED_CMD);
    OLED_WR_Byte(0xA1, OLED_CMD);
    OLED_WR_Byte(0xC8, OLED_CMD);
    OLED_WR_Byte(0xA6, OLED_CMD);
    OLED_WR_Byte(0xA8, OLED_CMD);
    OLED_WR_Byte(0x3f, OLED_CMD);
    OLED_WR_Byte(0xD3, OLED_CMD);
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0xd5, OLED_CMD);
    OLED_WR_Byte(0x80, OLED_CMD);
    OLED_WR_Byte(0xD9, OLED_CMD);
    OLED_WR_Byte(0xF1, OLED_CMD);
    OLED_WR_Byte(0xDA, OLED_CMD);
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD);
    OLED_WR_Byte(0x40, OLED_CMD);
    OLED_WR_Byte(0x20, OLED_CMD);
    OLED_WR_Byte(0x02, OLED_CMD);
    OLED_WR_Byte(0x8D, OLED_CMD);
    OLED_WR_Byte(0x14, OLED_CMD);
    OLED_WR_Byte(0xA4, OLED_CMD);
    OLED_WR_Byte(0xA6, OLED_CMD);
    OLED_Clear();
    OLED_WR_Byte(0xAF, OLED_CMD);
}

void OLED_Printf(uint8_t X, uint8_t Y, uint8_t FontSize, char *format, ...)
{
    char String[64];
    va_list arg;
    va_start(arg, format);
    vsnprintf(String, sizeof(String), format, arg);
    va_end(arg);
    OLED_ShowString(X, Y, (uint8_t *)String, FontSize);
}
