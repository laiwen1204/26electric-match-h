#include "q3.h"
#include "main.h"        /* HAL_GetTick */
#include "beam.h"
#include "vision.h"
#include "q4.h"
#include "bsp_can.h"     /* set_motor_current */
#include "task_sched.h"
#include <stdio.h>
#include <math.h>

/* ---- 视觉/执行器约束 ---- */
#define VISION_LEVEL_AFTER_MS      60U    /* 超过该时长无新样本: 摆杆缓回水平等待 */
#define VISION_TIMEOUT_MS         400U    /* 超过该时长无新样本: 锁存视觉故障 */

#define BALL_TARGET_POSITIVE_CM    5.0f
#define BALL_TARGET_NEGATIVE_CM   -5.0f
#define BALL_BEAM_SLEW_DEG_PER_STEP 0.60f /* 10 ms 控制步 */

#define Q3_POSITIVE_ENTRY_CM        4.00f
#define Q3_NEGATIVE_TOLERANCE_CM    0.90f
#define Q3_SETTLE_SPEED_CM_S        1.0f
#define Q3_NEGATIVE_SETTLE_MS       250U
#define Q3_SCORE_LIMIT_MS          5000U
#define Q3_HARD_LIMIT_MS          10000U

/*
 * 摄像头/摆杆链路约有 0.3s 延迟。用预测位置而不是等实测位置
 * 越过目标点才从加速切换到制动。
 */
#define Q3_PREDICT_TIME_S           0.65f
#define Q3_HOLD_PREDICT_TIME_S      0.32f
#define Q3_PREDICT_SPEED_LIMIT     12.0f
#define Q3_POS_BRAKE_SPEED_CM_S      2.5f
#define Q3_POS_REACCEL_PRED_CM      2.15f
#define Q3_POS_SPEED_BASE_CM_S        0.8f
#define Q3_POS_SPEED_STEP_CM_S        0.8f
#define Q3_POS_SPEED_MAX_CM_S         2.8f
#define Q3_POS_ACCEL_EFFORT_DEG     9.0f
#define Q3_POS_ACCEL_BOOST_STEP_DEG 1.0f
#define Q3_POS_ACCEL_BOOST_MS       350U
#define Q3_POS_ACCEL_EFFORT_MAX    12.0f
#define Q3_NEG_ACCEL_EFFORT_DEG    -7.5f
#define Q3_POS_BRAKE_EFFORT_DEG    -5.0f
#define Q3_NEG_BRAKE_EFFORT_DEG    11.0f
#define Q3_NEG_BRAKE_LOOKAHEAD_S     1.7f
#define Q3_NEG_BRAKE_PRED_CM       -5.0f
#define Q3_NEG_ACCEL_MAX_MS        1400U
#define Q3_NEG_BRAKE_MIN_MS         250U
#define Q3_CAPTURE_KP                1.4f
#define Q3_CAPTURE_KD                0.8f
#define Q3_ACTUATOR_FEEDBACK_K       0.0f
#define Q3_CAPTURE_EFFORT_MAX        6.0f
#define Q3_HOLD_DEADBAND_CM          0.90f
#define Q3_HOLD_SPEED_DEADBAND       1.50f
#define Q3_HOLD_PULSE_MS            220U
#define Q3_HOLD_COOLDOWN_MS         160U
#define Q3_CAPTURE_STALL_MS          150U
#define Q3_CAPTURE_PULSE_MS          280U
#define Q3_CAPTURE_COOLDOWN_MS       180U

typedef enum {
    Q3_IDLE = 0,
    Q3_TO_POSITIVE,
    Q3_TO_NEGATIVE,
    Q3_HOLD_NEGATIVE,
    Q3_TIMEOUT,
    Q3_VISION_LOST
} q3_state_t;

typedef enum {
    Q3_PHASE_NONE = 0,
    Q3_PHASE_POS_ACCEL,
    Q3_PHASE_POS_BRAKE,
    Q3_PHASE_NEG_ACCEL,
    Q3_PHASE_NEG_BRAKE,
    Q3_PHASE_NEG_CAPTURE,
    Q3_PHASE_CAPTURE_PULSE,
    Q3_PHASE_HOLD_PULSE
} q3_phase_t;

static q3_state_t q3_state;
static q3_phase_t q3_phase;
static float   q3_target_cm;
static uint32_t q3_start_ms;
static uint32_t q3_in_tolerance_since_ms;
static float   q3_predicted_position_cm;
static float   q3_last_effort_deg;
static float   q3_hold_pulse_effort_deg;
static uint32_t q3_hold_pulse_until_ms;
static uint32_t q3_hold_cooldown_until_ms;
static uint32_t q3_phase_start_ms;
static uint32_t q3_capture_still_since_ms;
static float   q3_capture_pulse_effort_deg;
static uint32_t q3_capture_pulse_until_ms;
static uint32_t q3_capture_cooldown_until_ms;
static uint8_t q3_positive_pulse_count;
static uint8_t q3_deadline_missed;

static void q3_set_target(float target_cm)
{
    q3_target_cm = target_cm;
    q3_in_tolerance_since_ms = 0U;
}

uint8_t q3_is_idle(void)
{
    return (q3_state == Q3_IDLE) ? 1U : 0U;
}

uint8_t q3_start(void)
{
    uint32_t now = HAL_GetTick();

    if (!q4_is_idle())
        return 0U;
    if (vision_last_sample_ms() == 0U
        || now - vision_last_sample_ms() >= VISION_LEVEL_AFTER_MS)
        return 0U;
    if (!beam_motor.position_zero_valid)
        return 0U;
    if (fabsf(vision_position_cm()) > 1.5f)
        return 0U;

    /* 连杆断电可反驱, 开机位置不固定: 启动时锁存实时角为本次中立点 */
    beam_capture_neutral();
    beam_output_enabled = 1U;
    q3_state = Q3_TO_POSITIVE;
    q3_phase = Q3_PHASE_POS_ACCEL;
    q3_start_ms = now;
    beam_angle_cmd_deg = beam_center_deg;
    vision_reset_speed();
    q3_predicted_position_cm = vision_position_cm();
    q3_last_effort_deg = 0.0f;
    q3_hold_pulse_effort_deg = 0.0f;
    q3_hold_pulse_until_ms = 0U;
    q3_hold_cooldown_until_ms = 0U;
    q3_phase_start_ms = now;
    q3_capture_still_since_ms = 0U;
    q3_capture_pulse_effort_deg = 0.0f;
    q3_capture_pulse_until_ms = 0U;
    q3_capture_cooldown_until_ms = 0U;
    q3_positive_pulse_count = 0U;
    q3_deadline_missed = 0U;
    q3_set_target(BALL_TARGET_POSITIVE_CM);
    task_pause(1); /* 轮询 printf 会引入数 ms 的控制抖动, 运行时停掉打印任务 */
    return 1U;
}

void q3_stop(void)
{
    q3_state = Q3_IDLE;
    q3_phase = Q3_PHASE_NONE;
    q3_target_cm = 0.0f;
    beam_idle();
    task_resume(1);
}

static float q3_predict_position(void)
{
    float speed = vision_speed_cm_s();

    if (speed > Q3_PREDICT_SPEED_LIMIT)
        speed = Q3_PREDICT_SPEED_LIMIT;
    if (speed < -Q3_PREDICT_SPEED_LIMIT)
        speed = -Q3_PREDICT_SPEED_LIMIT;
    return vision_position_cm() + Q3_PREDICT_TIME_S * speed;
}

void q3_control_task(void)
{
    float error;
    float effective_effort;
    float hold_signal;
    float actual_effort;
    float positive_accel_effort;
    float requested_angle;
    float angle_difference;
    uint32_t now;

    if (q3_state == Q3_IDLE)
        return;

    now = HAL_GetTick();
    if (vision_last_sample_ms() == 0U
        || now - vision_last_sample_ms() >= VISION_TIMEOUT_MS) {
        q3_state = Q3_VISION_LOST;
        q3_phase = Q3_PHASE_NONE;
        beam_idle();
        return;
    }
    if (q3_state == Q3_VISION_LOST) {
        /* 视觉故障锁存: 必须显式重新发 q3 才能再启动 */
        beam_output_enabled = 0U;
        set_motor_current(0, 0, 0, 0);
        return;
    }

    if (now - q3_start_ms > Q3_SCORE_LIMIT_MS
        && q3_state != Q3_HOLD_NEGATIVE)
        q3_deadline_missed = 1U;

    if (now - q3_start_ms > Q3_HARD_LIMIT_MS
        && q3_state != Q3_HOLD_NEGATIVE) {
        q3_state = Q3_TIMEOUT;
        q3_phase = Q3_PHASE_NONE;
        q3_last_effort_deg = 0.0f;
        beam_idle();
        task_resume(1);
        return;
    }
    if (q3_state == Q3_TIMEOUT) {
        beam_output_enabled = 0U;
        set_motor_current(0, 0, 0, 0);
        return;
    }

    /*
     * 容忍零星的 20ms 级检测丢帧。若连续数帧没有有效位置,
     * 不再改变轨迹, 摆杆缓回水平观察。400ms 断线在上面按锁存故障处理。
     */
    if (now - vision_last_sample_ms() >= VISION_LEVEL_AFTER_MS) {
        angle_difference = beam_center_deg - beam_angle_cmd_deg;
        if (angle_difference > BALL_BEAM_SLEW_DEG_PER_STEP)
            beam_angle_cmd_deg += BALL_BEAM_SLEW_DEG_PER_STEP;
        else if (angle_difference < -BALL_BEAM_SLEW_DEG_PER_STEP)
            beam_angle_cmd_deg -= BALL_BEAM_SLEW_DEG_PER_STEP;
        else
            beam_angle_cmd_deg = beam_center_deg;

        q3_last_effort_deg = 0.0f;
        beam_apply_position();
        return;
    }

    error = q3_target_cm - vision_position_cm();
    q3_predicted_position_cm = q3_predict_position();
    effective_effort = 0.0f;

    if (q3_state == Q3_TO_POSITIVE) {
        /*
         * 到达 +4.05cm 即证明 +5cm 点已进入允许的 1cm 误差带。
         * 立即保持反向倾斜, 开始返程长段。
         */
        if (vision_position_cm() >= Q3_POSITIVE_ENTRY_CM) {
            q3_state = Q3_TO_NEGATIVE;
            q3_phase = Q3_PHASE_NEG_ACCEL;
            q3_phase_start_ms = now;
            q3_set_target(BALL_TARGET_NEGATIVE_CM);
            effective_effort = Q3_NEG_ACCEL_EFFORT_DEG;
        } else {
            if (q3_phase == Q3_PHASE_POS_ACCEL
                && vision_speed_cm_s()
                   >= Q3_POS_BRAKE_SPEED_CM_S) {
                q3_phase = Q3_PHASE_POS_BRAKE;
            } else if (q3_phase == Q3_PHASE_POS_BRAKE
                     && vision_position_cm()
                        < Q3_POSITIVE_ENTRY_CM
                     && vision_speed_cm_s() <= 0.2f) {
                if (q3_positive_pulse_count < 3U)
                    q3_positive_pulse_count++;
                q3_phase = Q3_PHASE_POS_ACCEL;
                q3_phase_start_ms = now;
            }

            if (q3_phase == Q3_PHASE_POS_BRAKE) {
                effective_effort = Q3_POS_BRAKE_EFFORT_DEG;
            } else {
                positive_accel_effort =
                    Q3_POS_ACCEL_EFFORT_DEG
                    + Q3_POS_ACCEL_BOOST_STEP_DEG
                      * (float)((now - q3_phase_start_ms)
                                / Q3_POS_ACCEL_BOOST_MS);
                if (positive_accel_effort
                    > Q3_POS_ACCEL_EFFORT_MAX)
                    positive_accel_effort =
                        Q3_POS_ACCEL_EFFORT_MAX;
                effective_effort = positive_accel_effort;
            }
        }
    } else if (q3_state == Q3_TO_NEGATIVE) {
        if (q3_phase == Q3_PHASE_NEG_ACCEL
            && (vision_position_cm()
                + Q3_NEG_BRAKE_LOOKAHEAD_S * vision_speed_cm_s()
                <= Q3_NEG_BRAKE_PRED_CM
                || now - q3_phase_start_ms
                   >= Q3_NEG_ACCEL_MAX_MS)) {
            q3_phase = Q3_PHASE_NEG_BRAKE;
            q3_phase_start_ms = now;
        } else if (q3_phase == Q3_PHASE_NEG_BRAKE
                   && now - q3_phase_start_ms
                      >= Q3_NEG_BRAKE_MIN_MS
                   && vision_position_cm() <= -4.0f
                   && vision_speed_cm_s() >= -5.0f) {
            q3_phase = Q3_PHASE_NEG_CAPTURE;
            q3_phase_start_ms = now;
        } else if (q3_phase == Q3_PHASE_NEG_BRAKE
                   && now - q3_phase_start_ms
                      >= Q3_NEG_BRAKE_MIN_MS
                   && vision_position_cm() > -4.0f
                   && vision_speed_cm_s() >= -0.5f) {
            /*
             * 强制动可能让高摩擦的球在到达捕获区前停下。
             * 重新加速, 而不是让摆杆一直斜在错误方向。
             */
            q3_phase = Q3_PHASE_NEG_ACCEL;
            q3_phase_start_ms = now;
        }

        if (fabsf(error) <= Q3_NEGATIVE_TOLERANCE_CM
            && fabsf(vision_speed_cm_s()) <= Q3_SETTLE_SPEED_CM_S) {
            effective_effort = 0.0f;
            if (q3_in_tolerance_since_ms == 0U)
                q3_in_tolerance_since_ms = now;
            else if (now - q3_in_tolerance_since_ms
                     >= Q3_NEGATIVE_SETTLE_MS) {
                q3_state = Q3_HOLD_NEGATIVE;
                q3_phase = Q3_PHASE_NONE;
                q3_in_tolerance_since_ms = 0U;
            }
        } else {
            q3_in_tolerance_since_ms = 0U;
            if (q3_phase == Q3_PHASE_NEG_ACCEL) {
                effective_effort = Q3_NEG_ACCEL_EFFORT_DEG;
            } else if (q3_phase == Q3_PHASE_NEG_BRAKE) {
                effective_effort = Q3_NEG_BRAKE_EFFORT_DEG;
            } else {
                /*
                 * 破粘滞脉冲在整个持续时间内锁存输出。
                 * 不要被第一个噪声非零速度样本打断:
                 * 连杆需要时间才能到达有效角度。
                 */
                if (now < q3_capture_pulse_until_ms) {
                    effective_effort =
                        q3_capture_pulse_effort_deg;
                    q3_phase = Q3_PHASE_CAPTURE_PULSE;
                } else if (now < q3_capture_cooldown_until_ms) {
                    effective_effort = 0.0f;
                    q3_phase = Q3_PHASE_NEG_CAPTURE;
                    q3_capture_still_since_ms = 0U;
                } else {
                    q3_phase = Q3_PHASE_NEG_CAPTURE;
                    hold_signal = Q3_CAPTURE_KP * error
                                - Q3_CAPTURE_KD
                                  * vision_speed_cm_s();
                    actual_effort =
                        (beam_motor.pos_actual_deg
                         - beam_center_deg)
                        / beam_direction;
                    hold_signal -=
                        Q3_ACTUATOR_FEEDBACK_K * actual_effort;

                    if (fabsf(vision_speed_cm_s()) < 0.25f
                        && fabsf(error)
                           > Q3_HOLD_DEADBAND_CM) {
                        if (q3_capture_still_since_ms == 0U)
                            q3_capture_still_since_ms = now;
                    } else {
                        q3_capture_still_since_ms = 0U;
                    }

                    if (q3_capture_still_since_ms != 0U
                        && now - q3_capture_still_since_ms
                           >= Q3_CAPTURE_STALL_MS) {
                        q3_capture_pulse_effort_deg =
                            (error > 0.0f)
                            ? Q3_POS_ACCEL_EFFORT_DEG
                            : Q3_NEG_ACCEL_EFFORT_DEG;
                        q3_capture_pulse_until_ms =
                            now + Q3_CAPTURE_PULSE_MS;
                        q3_capture_cooldown_until_ms =
                            q3_capture_pulse_until_ms
                            + Q3_CAPTURE_COOLDOWN_MS;
                        q3_capture_still_since_ms = 0U;
                        effective_effort =
                            q3_capture_pulse_effort_deg;
                        q3_phase = Q3_PHASE_CAPTURE_PULSE;
                    } else {
                        effective_effort = hold_signal;
                        if (effective_effort
                            > Q3_CAPTURE_EFFORT_MAX)
                            effective_effort =
                                Q3_CAPTURE_EFFORT_MAX;
                        if (effective_effort
                            < -Q3_CAPTURE_EFFORT_MAX)
                            effective_effort =
                                -Q3_CAPTURE_EFFORT_MAX;
                    }
                }
            }
        }
    } else if (q3_state == Q3_HOLD_NEGATIVE) {
        /*
         * -5cm 附近连杆死区让小幅连续 PID 失效。
         * 用一次全幅短脉冲破死区, 然后回水平, 冷却期内观察。
         */
        if (now < q3_hold_pulse_until_ms) {
            effective_effort = q3_hold_pulse_effort_deg;
            q3_phase = Q3_PHASE_HOLD_PULSE;
        } else if (now < q3_hold_cooldown_until_ms) {
            effective_effort = 0.0f;
            q3_phase = Q3_PHASE_NONE;
        } else {
            hold_signal = error
                        - Q3_HOLD_PREDICT_TIME_S
                          * vision_speed_cm_s();
            if (fabsf(error) <= Q3_HOLD_DEADBAND_CM
                && fabsf(vision_speed_cm_s())
                   <= Q3_HOLD_SPEED_DEADBAND) {
                effective_effort = 0.0f;
                q3_phase = Q3_PHASE_NONE;
            } else if (fabsf(hold_signal) >= 0.35f) {
                q3_hold_pulse_effort_deg =
                    (hold_signal > 0.0f)
                    ? Q3_POS_ACCEL_EFFORT_DEG
                    : Q3_NEG_ACCEL_EFFORT_DEG;
                q3_hold_pulse_until_ms = now + Q3_HOLD_PULSE_MS;
                q3_hold_cooldown_until_ms =
                    q3_hold_pulse_until_ms + Q3_HOLD_COOLDOWN_MS;
                effective_effort = q3_hold_pulse_effort_deg;
                q3_phase = Q3_PHASE_HOLD_PULSE;
            } else {
                effective_effort = 0.0f;
                q3_phase = Q3_PHASE_NONE;
            }
        }
    } else {
        effective_effort = 0.0f;
    }

    q3_last_effort_deg = effective_effort;
    requested_angle = beam_clamp_angle(beam_center_deg
                                       + beam_direction * effective_effort);

    angle_difference = requested_angle - beam_angle_cmd_deg;
    if (angle_difference > BALL_BEAM_SLEW_DEG_PER_STEP)
        beam_angle_cmd_deg += BALL_BEAM_SLEW_DEG_PER_STEP;
    else if (angle_difference < -BALL_BEAM_SLEW_DEG_PER_STEP)
        beam_angle_cmd_deg -= BALL_BEAM_SLEW_DEG_PER_STEP;
    else
        beam_angle_cmd_deg = requested_angle;

    beam_apply_position();
}

void q3_print_status(void)
{
    uint32_t now = HAL_GetTick();

    printf("Q3,%lu,%u,%.2f,%.2f,%.2f,%.2f,%u,%lu,%.3f,%u,%.2f,%.2f,%u\r\n",
           (unsigned long)((q3_state == Q3_IDLE) ? 0U : now - q3_start_ms),
           (unsigned int)q3_state,
           q3_target_cm,
           vision_position_cm(),
           vision_speed_cm_s(),
           beam_angle_cmd_deg,
           vision_found(),
           (unsigned long)(now - vision_last_sample_ms()),
           beam_motor.pos_actual_deg,
           (unsigned int)q3_phase,
           q3_predicted_position_cm,
           q3_last_effort_deg,
           (unsigned int)q3_deadline_missed);
}
