#include "cmd.h"
#include "main.h"        /* HAL_GetTick */
#include "beam.h"
#include "vision.h"
#include "q3.h"
#include "q4.h"
#include "bsp_can.h"     /* motor_info / set_motor_current */
#include <stdio.h>
#include <string.h>

#define CMD_BUF_SIZE 32

static uint8_t  s_rx_byte;
static char     s_rx_build[CMD_BUF_SIZE];
static char     s_cmd_buf[CMD_BUF_SIZE];
static volatile uint8_t s_rx_index;
static volatile uint8_t s_cmd_ready;

uint8_t *cmd_rx_byte_addr(void)
{
    return &s_rx_byte;
}

void cmd_rx_byte(uint8_t b)
{
    uint8_t i;

    if (b == '\r' || b == '\n') {
        if (s_rx_index > 0 && !s_cmd_ready) {
            for (i = 0; i < s_rx_index; i++)
                s_cmd_buf[i] = s_rx_build[i];
            s_cmd_buf[s_rx_index] = '\0';
            s_cmd_ready = 1;
        }
        s_rx_index = 0;
    } else if (s_rx_index < CMD_BUF_SIZE - 1) {
        s_rx_build[s_rx_index++] = (char)b;
    }
}

void cmd_process(void)
{
    float value;

    if (!s_cmd_ready)
        return;

    if (strcmp(s_cmd_buf, "cam_status") == 0) {
        camera_print_status();
        goto done;
    }
    if (strcmp(s_cmd_buf, "cam_dump") == 0) {
        camera_dump_recent();
        goto done;
    }
    if (strcmp(s_cmd_buf, "q4") == 0) {
        q4_set_target(0.0f); /* 裸 q4 永远稳在中心 O */
        if (q4_start())
            printf("q4_started\r\n");
        else
            printf("q4_start_rejected,vision=%u,age=%lu,motor=%u,pos=%.2f\r\n",
                   vision_found(),
                   HAL_GetTick() - vision_last_rx_ms(),
                   beam_motor.position_zero_valid,
                   vision_position_cm());
        goto done;
    }
    if (strcmp(s_cmd_buf, "q4_stop") == 0) {
        q4_stop();
        printf("q4_stopped\r\n");
        goto done;
    }
    if (strcmp(s_cmd_buf, "q4_status") == 0) {
        q4_print_status();
        goto done;
    }
    if (sscanf(s_cmd_buf, "q4kp:%f", &value) == 1) {
        q4_set_kp(value);
        printf("q4kp=%.3f\r\n", (value < 0.0f) ? 0.0f : value);
        goto done;
    }
    if (sscanf(s_cmd_buf, "q4kd:%f", &value) == 1) {
        q4_set_kd(value);
        printf("q4kd=%.3f\r\n", (value < 0.0f) ? 0.0f : value);
        goto done;
    }
    if (sscanf(s_cmd_buf, "q4max:%f", &value) == 1) {
        printf("q4max=%.2f\r\n", q4_set_effort_max(value));
        goto done;
    }
    /* 要求6: 稳球在任意指定位置。"q6:<cm>" 一条命令完成设目标+启动 */
    if (sscanf(s_cmd_buf, "q6:%f", &value) == 1) {
        q4_set_target(value);
        if (q4_start())
            printf("q6_started,target=%.2f\r\n", q4_get_target());
        else
            printf("q6_start_rejected,vision=%u,age=%lu,motor=%u,pos=%.2f,target=%.2f\r\n",
                   vision_found(),
                   HAL_GetTick() - vision_last_rx_ms(),
                   beam_motor.position_zero_valid,
                   vision_position_cm(),
                   q4_get_target());
        goto done;
    }
    if (sscanf(s_cmd_buf, "q4target:%f", &value) == 1) {
        q4_set_target(value);
        printf("q4target=%.2f\r\n", q4_get_target());
        goto done;
    }
    if (strcmp(s_cmd_buf, "q3") == 0) {
        if (q3_start())
            printf("q3_started\r\n");
        else
            printf("q3_start_rejected,vision=%u,age=%lu,motor=%u,pos=%.2f\r\n",
                   vision_found(),
                   HAL_GetTick() - vision_last_rx_ms(),
                   beam_motor.position_zero_valid,
                   vision_position_cm());
        goto done;
    }
    if (strcmp(s_cmd_buf, "q3_stop") == 0) {
        q3_stop();
        printf("q3_stopped\r\n");
        goto done;
    }
    if (strcmp(s_cmd_buf, "q3_status") == 0) {
        q3_print_status();
        goto done;
    }
    if (sscanf(s_cmd_buf, "target_pos:%f", &value) == 1) {
        q3_stop();
        q4_stop();
        if (beam_motor.position_zero_valid) {
            beam_output_enabled = 1U;
            m2006_set_position(&beam_motor, value);
            printf("target_pos=%.2f\r\n", value);
        } else {
            printf("position_feedback_not_ready\r\n");
        }
        goto done;
    }
    if (strcmp(s_cmd_buf, "zero") == 0) {
        if (motor_info[0].can_id != 0) {
            m2006_zero_position(&beam_motor, motor_info[0].total_angle);
            printf("position_zeroed\r\n");
        } else {
            printf("position_feedback_not_ready\r\n");
        }
        goto done;
    }
    if (sscanf(s_cmd_buf, "poskp:%f", &value) == 1) {
        if (value < 0.0f) value = 0.0f;
        beam_motor.pos_kp = value;
        printf("poskp=%.3f\r\n", value);
        goto done;
    }
    if (sscanf(s_cmd_buf, "posdead:%f", &value) == 1) {
        if (value < 0.0f) value = 0.0f;
        beam_motor.pos_deadband = value;
        printf("posdead=%.3f\r\n", value);
        goto done;
    }
    if (sscanf(s_cmd_buf, "posmin:%f", &value) == 1) {
        if (value < 0.0f) value = 0.0f;
        beam_motor.pos_min_rpm = value;
        printf("posmin=%.2f\r\n", value);
        goto done;
    }
    if (sscanf(s_cmd_buf, "posmax:%f", &value) == 1) {
        if (value < beam_motor.pos_min_rpm)
            value = beam_motor.pos_min_rpm;
        beam_motor.pos_max_rpm = value;
        printf("posmax=%.2f\r\n", value);
        goto done;
    }
    if (sscanf(s_cmd_buf, "target_speed:%f", &value) == 1) {
        q3_stop();
        q4_stop();
        beam_output_enabled = 1U;
        m2006_set_speed(&beam_motor, value);
        printf("target_speed=%.2f\r\n", value);
        goto done;
    }
    if (sscanf(s_cmd_buf, "balldir:%f", &value) == 1) {
        beam_direction = (value < 0.0f) ? -1.0f : 1.0f;
        printf("balldir=%.0f\r\n", beam_direction);
        goto done;
    }
    if (sscanf(s_cmd_buf, "ballcenter:%f", &value) == 1) {
        if (value < -20.0f) value = -20.0f;
        if (value > 20.0f) value = 20.0f;
        beam_center_deg = value;
        printf("ballcenter=%.2f\r\n", value);
        goto done;
    }
    if (sscanf(s_cmd_buf, "ballmax:%f", &value) == 1) {
        if (value < 0.5f) value = 0.5f;
        if (value > 15.0f) value = 15.0f;
        beam_angle_max_deg = value;
        printf("ballmax=%.2f\r\n", value);
        goto done;
    }
    if (sscanf(s_cmd_buf, "kp:%f", &value) == 1) {
        beam_motor.speed_pid.kp = value;
        printf("kp=%.4f\r\n", value);
        goto done;
    }
    if (sscanf(s_cmd_buf, "ki:%f", &value) == 1) {
        beam_motor.speed_pid.ki = value;
        printf("ki=%.5f\r\n", value);
        goto done;
    }
    if (sscanf(s_cmd_buf, "ff:%f", &value) == 1) {
        beam_motor.ff_gain = value;
        printf("ff=%.4f\r\n", value);
        goto done;
    }
    if (sscanf(s_cmd_buf, "ks:%f", &value) == 1) {
        if (value < 0.0f) value = 0.0f;
        beam_motor.static_ff = value;
        printf("ks=%.1f\r\n", value);
        goto done;
    }
    if (sscanf(s_cmd_buf, "lpf:%f", &value) == 1) {
        if (value < 0.0f) value = 0.0f;
        if (value > 1.0f) value = 1.0f;
        beam_motor.filt_alpha = value;
        printf("lpf=%.3f\r\n", value);
        goto done;
    }
    if (sscanf(s_cmd_buf, "slew:%f", &value) == 1) {
        if (value < 0.0f) value = 0.0f;
        beam_motor.slew_rate = value;
        printf("slew=%.2f\r\n", value);
        goto done;
    }
    if (strcmp(s_cmd_buf, "stop") == 0) {
        q3_stop();
        q4_stop();
        m2006_stop(&beam_motor);
        set_motor_current(0, 0, 0, 0);
        printf("stop\r\n");
        goto done;
    }

    printf("unknown_cmd:%s\r\n", s_cmd_buf);

done:
    s_cmd_ready = 0;
}
