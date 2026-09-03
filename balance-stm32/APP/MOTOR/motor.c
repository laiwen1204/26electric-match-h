#include "motor.h"
#include <math.h>

#define PERIOD 8192.0f

static int is_in_forbidden(float p, float lo, float hi)
{
    float r = fmodf(p - lo, PERIOD);
    if (r < 0) r += PERIOD;
    return (r > 0.0f && r < (hi - lo));
}

static float clamp_to_safe(float pos, float lo, float hi)
{
    float r = fmodf(pos - lo, PERIOD);
    if (r < 0) r += PERIOD;
    float width = hi - lo;
    if (r <= 0.0f || r >= width) return pos;
    if (r <= width * 0.5f)
        return pos - r;
    else
        return pos + (width - r);
}

static int path_crosses_forbidden(float a, float b, float lo, float hi)
{
    float seg_lo = a < b ? a : b;
    float seg_hi = a < b ? b : a;
    int k_min = (int)floorf((seg_lo - hi) / PERIOD);
    int k_max = (int)ceilf((seg_hi - lo) / PERIOD);
    for (int k = k_min; k <= k_max; k++) {
        float z_lo = lo + k * PERIOD;
        float z_hi = hi + k * PERIOD;
        if (z_lo < seg_hi && z_hi > seg_lo)
            return 1;
    }
    return 0;
}

void motor_ctrl_init(motor_ctrl_t *m, uint8_t motor_type)
{
    float spd_out_max = (motor_type == MOTOR_TYPE_M3508) ? 16384.0f : 30000.0f;
    float spd_kp      = (motor_type == MOTOR_TYPE_M3508) ? 0.0f   : 8.0f;
    float spd_ki      = (motor_type == MOTOR_TYPE_M3508) ? 0.0f    : 0.2f;
    float default_ff  = (motor_type == MOTOR_TYPE_M3508) ? 3.0f    : 10.0f;

    pid_init(&m->speed_pid, spd_kp, spd_ki, 0.0f, 5000.0f, spd_out_max);
    pid_init(&m->pos_pid,   1.0f, 0.0f, 0.0f, 5000.0f, 350.0f);
    m->ff_gain      = default_ff;
    m->filt_speed   = 0.0f;
    m->filt_alpha   = 0.3f;
    m->target_pos   = 0.0f;
    m->pos_min      = 0.0f;
    m->pos_max      = 0.0f;
    m->limit_enable = 0;
    m->motor_type   = motor_type;
}

void motor_ctrl_set_target(motor_ctrl_t *m, float pos)
{
    if (m->limit_enable) {
        pos = clamp_to_safe(pos, m->pos_min, m->pos_max);
    }
    m->target_pos = pos;
    pid_clear(&m->pos_pid);
}

void motor_ctrl_set_limit(motor_ctrl_t *m, float min, float max)
{
    m->pos_min = min;
    m->pos_max = max;
    m->limit_enable = 1;
}

float motor_ctrl_step(motor_ctrl_t *m, float pos_fdb, float speed_fdb)
{
    m->filt_speed += m->filt_alpha * (speed_fdb - m->filt_speed);

    float effective_target = m->target_pos;
    if (m->limit_enable) {
        if (is_in_forbidden(pos_fdb, m->pos_min, m->pos_max)) {
            effective_target = clamp_to_safe(pos_fdb, m->pos_min, m->pos_max);
            pid_clear(&m->pos_pid);
        } else if (path_crosses_forbidden(pos_fdb, effective_target, m->pos_min, m->pos_max)) {
            effective_target += (effective_target > pos_fdb) ? -PERIOD : PERIOD;
        }
    }

    float speed_ref = pid_calc(&m->pos_pid, effective_target, pos_fdb);

    return pid_calc(&m->speed_pid, speed_ref, m->filt_speed)
           + m->ff_gain * speed_ref;
}
