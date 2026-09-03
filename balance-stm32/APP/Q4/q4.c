#include "q4.h"
#include "main.h"        /* HAL_GetTick */
#include "beam.h"
#include "vision.h"
#include "q3.h"
#include "task_sched.h"
#include <stdio.h>
#include <math.h>

/* ---- 视觉/执行器约束 ---- */
#define VISION_LEVEL_AFTER_MS      60U    /* 超过该时长无新样本: 摆杆缓回水平等待 */
#define VISION_TIMEOUT_MS         400U    /* 超过该时长无新样本: 进入视觉丢失态 */

#define Q4_TARGET_CM                0.0f
#define Q4_KP                       4.0f
#define Q4_KD                       3.3f
#define Q4_EFFORT_MAX_DEG          15.0f
#define Q4_DEADBAND_CM              0.50f
#define Q4_SPEED_DEADBAND           0.50f
#define Q4_NEAR_CM                  1.50f
#define Q4_KP_NEAR                  2.5f
#define Q4_KD_NEAR                  2.0f
#define Q4_MAX_NEAR_DEG             5.0f
#define Q4_BEAM_SLEW_DEG_PER_STEP   1.20f /* 10 ms 控制步, 120 deg/s */
#define Q4_STALL_MS                 180U
#define Q4_PULSE_MIN_MS             120U
#define Q4_PULSE_MAX_MS             800U
#define Q4_PULSE_BREAK_SPEED        0.50f
#define Q4_PULSE_BREAK_CM           0.20f
#define Q4_COOLDOWN_MS              250U
#define Q4_BRAKE_KD                 2.0f
#define Q4_BRAKE_MAX_DEG            8.0f
#define Q4_PULSE_EFFORT_DEG        15.0f
#define Q4_ANGLE_CAP_DEG           15.0f
#define Q4_START_TOLERANCE_CM       4.0f

typedef enum {
    Q4_IDLE = 0,
    Q4_RUN,
    Q4_VISION_LOST
} q4_state_t;

typedef enum {
    Q4_PHASE_NONE = 0,
    Q4_PHASE_PD,
    Q4_PHASE_PULSE,
    Q4_PHASE_COOLDOWN
} q4_phase_t;

static q4_state_t q4_state;
static q4_phase_t q4_phase;
static float   q4_kp = Q4_KP;
static float   q4_kd = Q4_KD;
static float   q4_effort_max_deg = Q4_EFFORT_MAX_DEG;
static float   q4_target_cm = Q4_TARGET_CM; /* 要求6: 任意停球位置 */
static float   q4_last_effort_deg;
static float   q4_pulse_effort_deg;
static uint32_t q4_start_ms;
static uint32_t q4_still_since_ms;
static uint32_t q4_pulse_start_ms;
static float   q4_pulse_start_pos_cm;
static uint32_t q4_cooldown_until_ms;

uint8_t q4_is_idle(void)
{
    return (q4_state == Q4_IDLE) ? 1U : 0U;
}

void q4_set_target(float cm)
{
    if (cm > 6.0f) cm = 6.0f;
    if (cm < -6.0f) cm = -6.0f;
    q4_target_cm = cm;
}

float q4_get_target(void)
{
    return q4_target_cm;
}

void q4_set_kp(float kp)
{
    if (kp < 0.0f) kp = 0.0f;
    q4_kp = kp;
}

void q4_set_kd(float kd)
{
    if (kd < 0.0f) kd = 0.0f;
    q4_kd = kd;
}

float q4_set_effort_max(float deg)
{
    if (deg < 0.5f) deg = 0.5f;
    if (deg > beam_angle_max_deg) deg = beam_angle_max_deg;
    q4_effort_max_deg = deg;
    return q4_effort_max_deg;
}

uint8_t q4_start(void)
{
    uint32_t now = HAL_GetTick();

    if (q4_state != Q4_IDLE)
        return 1U; /* 已在运行: 保持已锁存的中立点, 忽略重复启动 */
    if (!q3_is_idle())
        return 0U;
    if (vision_last_sample_ms() == 0U
        || now - vision_last_sample_ms() >= VISION_LEVEL_AFTER_MS)
        return 0U;
    if (!beam_motor.position_zero_valid)
        return 0U;
    if (fabsf(vision_position_cm() - q4_target_cm) > Q4_START_TOLERANCE_CM)
        return 0U;

    /* 与 q3 相同的中立点锁存: 操作员已调平摆杆、球放 O 点 */
    beam_capture_neutral();
    /* 小车发车需要持续倾斜来抵消加速度 */
    if (beam_angle_max_deg < Q4_ANGLE_CAP_DEG)
        beam_angle_max_deg = Q4_ANGLE_CAP_DEG;
    beam_output_enabled = 1U;
    q4_state = Q4_RUN;
    q4_phase = Q4_PHASE_PD;
    q4_start_ms = now;
    beam_angle_cmd_deg = beam_center_deg;
    vision_reset_speed();
    q4_last_effort_deg = 0.0f;
    q4_pulse_effort_deg = 0.0f;
    q4_still_since_ms = 0U;
    q4_pulse_start_ms = 0U;
    q4_pulse_start_pos_cm = 0.0f;
    q4_cooldown_until_ms = 0U;
    task_pause(1); /* 轮询 printf 会引入数 ms 的控制抖动, 运行时停掉打印任务 */
    return 1U;
}

void q4_stop(void)
{
    q4_state = Q4_IDLE;
    q4_phase = Q4_PHASE_NONE;
    beam_idle();
    task_resume(1);
}

void q4_control_task(void)
{
    float error;
    float signal;
    float effort;
    float requested_angle;
    float angle_difference;
    uint32_t now;

    if (q4_state == Q4_IDLE)
        return;

    now = HAL_GetTick();
    if (vision_last_sample_ms() == 0U
        || now - vision_last_sample_ms() >= VISION_TIMEOUT_MS) {
        /*
         * 要求4 行驶中摄像头断线: 不切断电机。
         * 保持摆杆水平(位置环保持), 让球尽量少滚,
         * 视觉数据一恢复就自动继续平衡。
         * 行驶震动下短时丢帧是常态; 这里若锁存故障,
         * 球将在剩余赛程里完全失控。
         */
        q4_state = Q4_VISION_LOST;
        q4_phase = Q4_PHASE_NONE;
        q4_pulse_start_ms = 0U;
        q4_cooldown_until_ms = 0U;
        q4_still_since_ms = 0U;
        beam_angle_cmd_deg = beam_center_deg;
        beam_output_enabled = 1U;
        beam_apply_position();
        return;
    }
    if (q4_state == Q4_VISION_LOST) {
        /* 视觉恢复: 从当前状态继续平衡 */
        q4_state = Q4_RUN;
        q4_phase = Q4_PHASE_PD;
        q4_still_since_ms = 0U;
        vision_reset_speed();
    }

    /* 短时检测丢帧: 摆杆缓回水平等待, 与 q3 的安全行为一致 */
    if (now - vision_last_sample_ms() >= VISION_LEVEL_AFTER_MS) {
        angle_difference = beam_center_deg - beam_angle_cmd_deg;
        if (angle_difference > Q4_BEAM_SLEW_DEG_PER_STEP)
            beam_angle_cmd_deg += Q4_BEAM_SLEW_DEG_PER_STEP;
        else if (angle_difference < -Q4_BEAM_SLEW_DEG_PER_STEP)
            beam_angle_cmd_deg -= Q4_BEAM_SLEW_DEG_PER_STEP;
        else
            beam_angle_cmd_deg = beam_center_deg;

        q4_last_effort_deg = 0.0f;
        beam_apply_position();
        return;
    }

    error = q4_target_cm - vision_position_cm();
    effort = 0.0f;

    if (q4_pulse_start_ms != 0U) {
        /*
         * 运动终止式破粘滞脉冲: 保持全幅输出直到球真的动了
         * (检测到速度或位移), 带硬上限。
         * 固定宽度脉冲已被证明无法逃出凹槽上的局部粘滞点。
         */
        uint32_t pulse_elapsed = now - q4_pulse_start_ms;

        if (pulse_elapsed >= Q4_PULSE_MIN_MS
            && (fabsf(vision_speed_cm_s()) > Q4_PULSE_BREAK_SPEED
                || fabsf(vision_position_cm() - q4_pulse_start_pos_cm)
                   > Q4_PULSE_BREAK_CM)) {
            q4_pulse_start_ms = 0U;
            q4_cooldown_until_ms = now + Q4_COOLDOWN_MS;
            q4_phase = Q4_PHASE_COOLDOWN;
        } else if (pulse_elapsed >= Q4_PULSE_MAX_MS) {
            q4_pulse_start_ms = 0U;
            q4_cooldown_until_ms = now + Q4_COOLDOWN_MS;
            q4_phase = Q4_PHASE_COOLDOWN;
        } else {
            effort = q4_pulse_effort_deg;
            q4_phase = Q4_PHASE_PULSE;
        }
    } else if (now < q4_cooldown_until_ms) {
        /*
         * 破粘滞脉冲后的主动阻尼: 泄掉脉冲带来的动量,
         * 防止球冲过目标在另一侧又触发脉冲(极限环泵振)。
         */
        effort = -Q4_BRAKE_KD * vision_speed_cm_s();
        if (effort > Q4_BRAKE_MAX_DEG)
            effort = Q4_BRAKE_MAX_DEG;
        if (effort < -Q4_BRAKE_MAX_DEG)
            effort = -Q4_BRAKE_MAX_DEG;
        q4_phase = Q4_PHASE_COOLDOWN;
        q4_still_since_ms = 0U;
    } else {
        q4_phase = Q4_PHASE_PD;

        /* 球静止在容差带内: 保持摆杆水平 */
        if (fabsf(error) <= Q4_DEADBAND_CM
            && fabsf(vision_speed_cm_s()) <= Q4_SPEED_DEADBAND) {
            q4_still_since_ms = 0U;
        } else {
            /*
             * 双区增益调度。远离 O 点需要全幅输出快速追球(发车),
             * 靠近 O 点时 0.3s 摄像头延迟会把大增益变成持续极限环,
             * 所以在 Q4_NEAR_CM 内向温和增益渐变。
             */
            float blend = fabsf(error) / Q4_NEAR_CM;
            float kp_eff;
            float kd_eff;
            float max_eff;

            if (blend > 1.0f) blend = 1.0f;
            kp_eff = Q4_KP_NEAR + (q4_kp - Q4_KP_NEAR) * blend;
            kd_eff = Q4_KD_NEAR + (q4_kd - Q4_KD_NEAR) * blend;
            max_eff = Q4_MAX_NEAR_DEG
                      + (q4_effort_max_deg - Q4_MAX_NEAR_DEG) * blend;

            signal = kp_eff * error - kd_eff * vision_speed_cm_s();
            effort = signal;
            if (effort > max_eff)
                effort = max_eff;
            if (effort < -max_eff)
                effort = -max_eff;

            /*
             * 连杆死区: 停在死区外的球对小幅连续输出永远没有响应。
             * 静止超过 Q4_STALL_MS 后打一次全幅脉冲, 然后进入冷却。
             */
            if (fabsf(vision_speed_cm_s()) < 0.25f
                && fabsf(error) > 0.80f) {
                if (q4_still_since_ms == 0U)
                    q4_still_since_ms = now;
                else if (now - q4_still_since_ms >= Q4_STALL_MS) {
                    q4_pulse_effort_deg =
                        (error > 0.0f)
                        ? Q4_PULSE_EFFORT_DEG
                        : -Q4_PULSE_EFFORT_DEG;
                    q4_pulse_start_ms = now;
                    q4_pulse_start_pos_cm = vision_position_cm();
                    q4_still_since_ms = 0U;
                    effort = q4_pulse_effort_deg;
                    q4_phase = Q4_PHASE_PULSE;
                }
            } else {
                q4_still_since_ms = 0U;
            }
        }
    }

    q4_last_effort_deg = effort;
    requested_angle = beam_clamp_angle(beam_center_deg
                                       + beam_direction * effort);

    angle_difference = requested_angle - beam_angle_cmd_deg;
    if (angle_difference > Q4_BEAM_SLEW_DEG_PER_STEP)
        beam_angle_cmd_deg += Q4_BEAM_SLEW_DEG_PER_STEP;
    else if (angle_difference < -Q4_BEAM_SLEW_DEG_PER_STEP)
        beam_angle_cmd_deg -= Q4_BEAM_SLEW_DEG_PER_STEP;
    else
        beam_angle_cmd_deg = requested_angle;

    beam_apply_position();
}

void q4_print_status(void)
{
    uint32_t now = HAL_GetTick();

    printf("Q4,%lu,%u,%.2f,%.2f,%.2f,%u,%lu,%.3f,%u,%.2f\r\n",
           (unsigned long)((q4_state == Q4_IDLE) ? 0U : now - q4_start_ms),
           (unsigned int)q4_state,
           vision_position_cm(),
           vision_speed_cm_s(),
           beam_angle_cmd_deg,
           vision_found(),
           (unsigned long)(now - vision_last_sample_ms()),
           beam_motor.pos_actual_deg,
           (unsigned int)q4_phase,
           q4_last_effort_deg);
}
