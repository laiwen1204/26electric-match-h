#ifndef M2006_H
#define M2006_H

#include "pid.h"
#include <stdint.h>

/*
 * M2006 + C610 defaults.
 * Confirm the actual gearbox before use if it is not the standard 36:1 unit.
 */
#define M2006_GEAR_RATIO       36.0f
#define M2006_ENCODER_CPR      8192.0f
#define M2006_CURRENT_MAX      10000.0f

typedef struct {
    pid_struct_t speed_pid;

    float ff_gain;
    float static_ff;

    float cmd_rpm;        /* user command, output-shaft rpm */
    float target_rpm;     /* ramped output-shaft rpm        */
    float actual_rpm;     /* filtered output-shaft rpm      */
    float filt_spd;       /* filtered rotor rpm             */
    float filt_alpha;
    float slew_rate;      /* output rpm per millisecond     */

    float pos_target_deg; /* relative output-shaft degrees  */
    float pos_actual_deg;
    float pos_zero_count; /* rotor-side accumulated counts  */
    float pos_kp;         /* output rpm / output degree     */
    float pos_deadband;
    float pos_min_rpm;
    float pos_max_rpm;

    uint8_t position_enable;
    uint8_t position_at_target;
    uint8_t position_zero_valid;
} m2006_t;

void  m2006_init(m2006_t *m);
void  m2006_set_speed(m2006_t *m, float rpm);
void  m2006_stop(m2006_t *m);

void  m2006_zero_position(m2006_t *m, int32_t total_angle);
void  m2006_set_position(m2006_t *m, float position_deg);
void  m2006_position_update(m2006_t *m, int32_t total_angle);

float m2006_update(m2006_t *m, int16_t rotor_speed);
float m2006_actual_rpm(const m2006_t *m);

#endif
