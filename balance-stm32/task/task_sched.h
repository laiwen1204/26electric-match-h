#ifndef TASK_SCHED_H
#define TASK_SCHED_H

#include <stdint.h>

typedef void (*task_func_t)(void);

typedef struct {
    task_func_t func;
    uint16_t    interval_ms;   /* period in ms           */
    uint16_t    counter;       /* internal countdown     */
    uint8_t     enable;        /* 1=running, 0=paused    */
} task_t;

void task_sched_init(void);
void task_sched_run(void);           /* call in main loop */
void task_add(uint8_t id, task_func_t func, uint16_t interval_ms);
void task_pause(uint8_t id);
void task_resume(uint8_t id);
void task_set_interval(uint8_t id, uint16_t interval_ms);
void task_tick(void);               /* call from TIM ISR */

#endif
