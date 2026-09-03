#ifndef __MOTOR_H
#define __MOTOR_H

#include "pid.h"
#include "bsp_can.h"

typedef struct {
    pid_struct_t speed_pid;
    pid_struct_t pos_pid;
    float ff_gain;
    float filt_speed;
    float filt_alpha;
    float target_pos;
    float pos_min;
    float pos_max;
    uint8_t limit_enable;
    uint8_t motor_type;
} motor_ctrl_t;

void motor_ctrl_init(motor_ctrl_t *m, uint8_t motor_type);
void motor_ctrl_set_target(motor_ctrl_t *m, float pos);
void motor_ctrl_set_limit(motor_ctrl_t *m, float min, float max);
float motor_ctrl_step(motor_ctrl_t *m, float pos_fdb, float speed_fdb);

#endif
