#include "vision.h"
#include "main.h"    /* HAL_GetTick */
#include "usart.h"   /* huart6 (camera_print_status 里读 RxState) */
#include <stdio.h>

#define CAMERA_RX_BUF_SIZE   256
#define CAMERA_RECENT_SIZE   16
#define VISION_LINE_BUF_SIZE 32

/* 球速度一阶低通系数: 位置差分噪声大, 0.35 在响应和平滑间折中 */
#define BALL_SPEED_FILTER_ALPHA 0.35f

/* ---- UART6 环形缓冲(中断写, 主循环读) ---- */
static uint8_t  camera_rx_byte_;
static uint8_t  camera_rx_buf[CAMERA_RX_BUF_SIZE];
static volatile uint16_t camera_rx_head;
static volatile uint16_t camera_rx_tail;
static volatile uint32_t camera_rx_overflow;
static volatile uint32_t camera_rx_total;
static volatile uint32_t camera_uart_error_total;
static volatile uint32_t camera_uart_last_error;
static volatile uint8_t  camera_last_byte;
static uint8_t  camera_recent[CAMERA_RECENT_SIZE];
static volatile uint8_t  camera_recent_head;

/* ---- 行解析状态 ---- */
static char    vision_line[VISION_LINE_BUF_SIZE];
static uint8_t vision_line_index;

/* ---- 球位状态(vision_accept_line 更新, q3/q4 控制环读取) ---- */
static volatile uint8_t  s_found;
static float   s_position_cm;
static float   s_speed_cm_s;
static float   s_last_position_cm;
static uint32_t s_last_rx_ms;
static uint32_t s_last_sample_ms;
static uint32_t s_valid_line_total;
static uint32_t s_invalid_line_total;

/* UART6 RX 中断回调里调用: 启动时由 main.c 把本变量的地址交给 HAL */
uint8_t *vision_rx_byte_addr(void)
{
    return &camera_rx_byte_;
}

void vision_rx_byte(uint8_t b)
{
    uint16_t next = (uint16_t)((camera_rx_head + 1U) % CAMERA_RX_BUF_SIZE);

    camera_rx_total++;
    camera_last_byte = b;
    camera_recent[camera_recent_head] = b;
    camera_recent_head = (uint8_t)((camera_recent_head + 1U) % CAMERA_RECENT_SIZE);
    if (next != camera_rx_tail) {
        camera_rx_buf[camera_rx_head] = b;
        camera_rx_head = next;
    } else {
        camera_rx_overflow++;
    }
}

void vision_rx_error(uint32_t err)
{
    camera_uart_error_total++;
    camera_uart_last_error = err;
}

static void vision_accept_line(void)
{
    int found;
    float position;
    uint32_t now;

    vision_line[vision_line_index] = '\0';
    if (sscanf(vision_line, "%d,%f", &found, &position) != 2) {
        s_invalid_line_total++;
        return;
    }
    if ((found != 0 && found != 1) || position < -30.0f || position > 30.0f) {
        s_invalid_line_total++;
        return;
    }

    now = HAL_GetTick();
    if (found) {
        if (s_last_sample_ms != 0U) {
            uint32_t dt_ms = now - s_last_sample_ms;

            if (dt_ms >= 5U && dt_ms <= 500U) {
                float raw_speed =
                    (position - s_last_position_cm) * 1000.0f
                    / (float)dt_ms;
                s_speed_cm_s += BALL_SPEED_FILTER_ALPHA
                                * (raw_speed - s_speed_cm_s);
            } else {
                s_speed_cm_s = 0.0f;
            }
        } else {
            s_speed_cm_s = 0.0f;
        }
        s_last_position_cm = position;
        s_last_sample_ms = now;
        s_position_cm = position;
    }

    s_found = (uint8_t)found;
    s_last_rx_ms = now;
    s_valid_line_total++;
}

void vision_parse_task(void)
{
    uint16_t tail = camera_rx_tail;

    while (tail != camera_rx_head) {
        uint8_t byte = camera_rx_buf[tail];
        tail = (uint16_t)((tail + 1U) % CAMERA_RX_BUF_SIZE);

        if (byte == '\r' || byte == '\n') {
            if (vision_line_index > 0U)
                vision_accept_line();
            vision_line_index = 0U;
        } else if (vision_line_index < VISION_LINE_BUF_SIZE - 1U) {
            vision_line[vision_line_index++] = (char)byte;
        } else {
            vision_line_index = 0U;
        }
    }
    camera_rx_tail = tail;
}

uint8_t vision_found(void)
{
    return s_found;
}

float vision_position_cm(void)
{
    return s_position_cm;
}

float vision_speed_cm_s(void)
{
    return s_speed_cm_s;
}

uint32_t vision_last_sample_ms(void)
{
    return s_last_sample_ms;
}

uint32_t vision_last_rx_ms(void)
{
    return s_last_rx_ms;
}

void vision_reset_speed(void)
{
    s_speed_cm_s = 0.0f;
}

void camera_print_status(void)
{
    printf("CAM,%lu,%u,%u,%lu,%lu,%lu,%lu,%u,%02X,%08lX,%02lX\r\n",
           (unsigned long)camera_rx_total,
           (unsigned int)camera_rx_head,
           (unsigned int)camera_rx_tail,
           (unsigned long)camera_rx_overflow,
           (unsigned long)s_valid_line_total,
           (unsigned long)s_invalid_line_total,
           (unsigned long)camera_uart_error_total,
           (unsigned int)vision_line_index,
           (unsigned int)camera_last_byte,
           (unsigned long)camera_uart_last_error,
           (unsigned long)huart6.RxState);
}

void camera_dump_recent(void)
{
    uint8_t i;
    uint8_t head = camera_recent_head;

    printf("CAMHEX");
    for (i = 0U; i < CAMERA_RECENT_SIZE; i++) {
        uint8_t index = (uint8_t)((head + i) % CAMERA_RECENT_SIZE);
        printf(",%02X", camera_recent[index]);
    }
    printf("\r\n");
}
