#include "m2006.h"
#include <math.h>

/*
 * These are conservative initial values, not tuned values.
 * Retune them on the actual M2006, C610 and mechanical load.
 */
#define M2006_DEFAULT_KP             1.5f
#define M2006_DEFAULT_KI             0.0f
#define M2006_DEFAULT_FF             0.10f
#define M2006_DEFAULT_STATIC_FF      145.0f
#define M2006_DEFAULT_ALPHA          0.25f
#define M2006_DEFAULT_SLEW           1.0f
#define M2006_STATIC_TRANSITION_RPM  5.0f

#define M2006_DEFAULT_POS_KP         1.0f
#define M2006_DEFAULT_POS_DEADBAND   0.15f
#define M2006_DEFAULT_POS_MIN_RPM    3.0f
#define M2006_DEFAULT_POS_MAX_RPM    15.0f
#define M2006_POSITION_EXIT_RATIO    2.0f

static float m2006_clampf(float value, float minimum, float maximum)
{
    if (value > maximum) return maximum;
    if (value < minimum) return minimum;
    return value;
}

static void m2006_clear_speed_state(m2006_t *m)
{
    m->cmd_rpm = 0.0f;
    m->target_rpm = 0.0f;
    pid_clear(&m->speed_pid);
}

void m2006_init(m2006_t *m)
{
    pid_init(&m->speed_pid,
             M2006_DEFAULT_KP,
             M2006_DEFAULT_KI,
             0.0f,
             5000.0f,
             M2006_CURRENT_MAX);

    m->ff_gain = M2006_DEFAULT_FF;
    m->static_ff = M2006_DEFAULT_STATIC_FF;
    m->cmd_rpm = 0.0f;
    m->target_rpm = 0.0f;
    m->actual_rpm = 0.0f;
    m->filt_spd = 0.0f;
    m->filt_alpha = M2006_DEFAULT_ALPHA;
    m->slew_rate = M2006_DEFAULT_SLEW;

    m->pos_target_deg = 0.0f;
    m->pos_actual_deg = 0.0f;
    m->pos_zero_count = 0.0f;
    m->pos_kp = M2006_DEFAULT_POS_KP;
    m->pos_deadband = M2006_DEFAULT_POS_DEADBAND;
    m->pos_min_rpm = M2006_DEFAULT_POS_MIN_RPM;
    m->pos_max_rpm = M2006_DEFAULT_POS_MAX_RPM;
    m->position_enable = 0;
    m->position_at_target = 1;
    m->position_zero_valid = 0;
}

void m2006_set_speed(m2006_t *m, float rpm)
{
    m->position_enable = 0;
    m->cmd_rpm = rpm;
}

void m2006_stop(m2006_t *m)
{
    m->position_enable = 0;
    m2006_clear_speed_state(m);
}

void m2006_zero_position(m2006_t *m, int32_t total_angle)
{
    m->pos_zero_count = (float)total_angle;
    m->pos_actual_deg = 0.0f;
    m->pos_target_deg = 0.0f;
    m->position_zero_valid = 1;
    m->position_at_target = 1;
    m2006_clear_speed_state(m);
}

void m2006_set_position(m2006_t *m, float position_deg)
{
    if (!m->position_zero_valid) return;

    m->pos_target_deg = position_deg;
    m->position_enable = 1;
    m->position_at_target = 0;
    pid_clear(&m->speed_pid);
}

void m2006_position_update(m2006_t *m, int32_t total_angle)
{
    float error;
    float abs_error;
    float speed_ref;

    if (!m->position_zero_valid) return;

    m->pos_actual_deg = ((float)total_angle - m->pos_zero_count)
                      * 360.0f
                      / (M2006_ENCODER_CPR * M2006_GEAR_RATIO);

    if (!m->position_enable) return;

    error = m->pos_target_deg - m->pos_actual_deg;
    abs_error = fabsf(error);

    if (m->position_at_target) {
        if (abs_error <= m->pos_deadband * M2006_POSITION_EXIT_RATIO) {
            m2006_clear_speed_state(m);
            return;
        }
        m->position_at_target = 0;
    }

    if (abs_error <= m->pos_deadband) {
        m->position_at_target = 1;
        m2006_clear_speed_state(m);
        return;
    }

    speed_ref = m2006_clampf(m->pos_kp * error,
                            -m->pos_max_rpm,
                            m->pos_max_rpm);

    if (speed_ref > 0.0f && speed_ref < m->pos_min_rpm)
        speed_ref = m->pos_min_rpm;
    else if (speed_ref < 0.0f && speed_ref > -m->pos_min_rpm)
        speed_ref = -m->pos_min_rpm;

    m->cmd_rpm = speed_ref;
}

float m2006_update(m2006_t *m, int16_t rotor_speed)
{
    float alpha;
    float rotor_target;
    float output;

    if (m->slew_rate > 0.0f && m->slew_rate < 1e6f) {
        float difference = m->cmd_rpm - m->target_rpm;

        if (difference > m->slew_rate)
            m->target_rpm += m->slew_rate;
        else if (difference < -m->slew_rate)
            m->target_rpm -= m->slew_rate;
        else
            m->target_rpm = m->cmd_rpm;
    } else {
        m->target_rpm = m->cmd_rpm;
    }

    alpha = m2006_clampf(m->filt_alpha, 0.0f, 1.0f);
    m->filt_spd = alpha * (float)rotor_speed
                + (1.0f - alpha) * m->filt_spd;
    m->actual_rpm = m->filt_spd / M2006_GEAR_RATIO;

    rotor_target = m->target_rpm * M2006_GEAR_RATIO;
    output = pid_calc(&m->speed_pid, rotor_target, m->filt_spd);
    output += m->ff_gain * rotor_target;
    output += m->static_ff * rotor_target
            / (fabsf(rotor_target) + M2006_STATIC_TRANSITION_RPM);

    return m2006_clampf(output, -M2006_CURRENT_MAX, M2006_CURRENT_MAX);
}

float m2006_actual_rpm(const m2006_t *m)
{
    return m->actual_rpm;
}
