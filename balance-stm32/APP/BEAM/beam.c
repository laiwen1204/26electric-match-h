#include "beam.h"
#include "bsp_can.h"   /* motor_info / can_cnt / set_motor_current */
#include <stdio.h>

/* CAN 反馈超时: 超过该时长收不到电调回报即切断输出(防失控) */
#define M2006_CAN_TIMEOUT_MS 100

m2006_t beam_motor;
uint8_t beam_output_enabled;
float   beam_center_deg    = 0.0f;
float   beam_direction     = -1.0f;   /* 当前机械安装方向: 负极性 */
float   beam_angle_max_deg = 12.0f;
float   beam_angle_cmd_deg;

static int16_t  s_last_output;
static uint16_t s_last_can_cnt;
static uint16_t s_can_stale_ms;

void beam_control_task(void)
{
    if (motor_info[0].can_id == 0) {
        s_last_output = 0;
        set_motor_current(0, 0, 0, 0);
        return;
    }

    if (!beam_motor.position_zero_valid)
        m2006_zero_position(&beam_motor, motor_info[0].total_angle);

    if (can_cnt != s_last_can_cnt) {
        s_last_can_cnt = can_cnt;
        s_can_stale_ms = 0;
    } else if (s_can_stale_ms < M2006_CAN_TIMEOUT_MS) {
        s_can_stale_ms++;
    }

    if (s_can_stale_ms >= M2006_CAN_TIMEOUT_MS) {
        m2006_stop(&beam_motor);
        s_last_output = 0;
        set_motor_current(0, 0, 0, 0);
        return;
    }

    m2006_position_update(&beam_motor, motor_info[0].total_angle);
    if (!beam_output_enabled) {
        s_last_output = 0;
        set_motor_current(0, 0, 0, 0);
        return;
    }
    s_last_output = (int16_t)m2006_update(&beam_motor, motor_info[0].rotor_speed);
    set_motor_current(s_last_output, 0, 0, 0);
}

void beam_print_task(void)
{
    printf("%.2f,%.2f,%.2f,%d,%d,%d,%.3f,%.3f,%u\r\n",
           beam_motor.cmd_rpm,
           beam_motor.target_rpm,
           m2006_actual_rpm(&beam_motor),
           motor_info[0].rotor_speed,
           motor_info[0].torque_current,
           s_last_output,
           beam_motor.pos_target_deg,
           beam_motor.pos_actual_deg,
           beam_motor.position_enable);
}

void beam_capture_neutral(void)
{
    /*
     * 连杆机构在断电状态下可反驱, 搬动摆杆会改变绝对编码器位置。
     * 题目开始时操作员已把球放在 O 点、摆杆调平, 此时锁存电机实时角
     * 作为本次运行的连杆中立点。
     */
    beam_center_deg = beam_motor.pos_actual_deg;
}

void beam_apply_position(void)
{
    /*
     * 不要每 10ms 调 m2006_set_position(): 那个 API 会清空内层速度环。
     * 直接原地更新移动中的位置目标。
     */
    beam_motor.pos_target_deg = beam_angle_cmd_deg;
    beam_motor.position_enable = 1U;
    beam_motor.position_at_target = 0U;
}

void beam_idle(void)
{
    beam_angle_cmd_deg = beam_motor.pos_actual_deg;
    beam_output_enabled = 0U;
    m2006_stop(&beam_motor);
    set_motor_current(0, 0, 0, 0);
}

float beam_clamp_angle(float requested_deg)
{
    if (requested_deg > beam_center_deg + beam_angle_max_deg)
        requested_deg = beam_center_deg + beam_angle_max_deg;
    if (requested_deg < beam_center_deg - beam_angle_max_deg)
        requested_deg = beam_center_deg - beam_angle_max_deg;
    return requested_deg;
}
