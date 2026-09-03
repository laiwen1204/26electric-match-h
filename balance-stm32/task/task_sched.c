#include "task_sched.h"

#define MAX_TASKS  16

static task_t  tasks[MAX_TASKS];
static uint8_t task_count;
volatile uint8_t sched_tick;   /* set by TIM ISR */

void task_sched_init(void)
{
    task_count = 0;
    sched_tick = 0;
    for (uint8_t i = 0; i < MAX_TASKS; i++)
    {
        tasks[i].func = ((void *)0);
        tasks[i].enable = 0;
    }
}

/* called from TIM4 ISR, 1kHz */
void task_tick(void)
{
    sched_tick = 1;
}

void task_add(uint8_t id, task_func_t func, uint16_t interval_ms)
{
    if (id >= MAX_TASKS || func == ((void *)0)) return;

    tasks[id].func        = func;
    tasks[id].interval_ms = interval_ms;
    tasks[id].counter     = interval_ms;
    tasks[id].enable      = 1;
    if (id >= task_count) task_count = id + 1;
}

void task_pause(uint8_t id)
{
    if (id < MAX_TASKS) tasks[id].enable = 0;
}

void task_resume(uint8_t id)
{
    if (id < MAX_TASKS && tasks[id].func)
    {
        tasks[id].enable  = 1;
        tasks[id].counter = tasks[id].interval_ms;
    }
}

void task_set_interval(uint8_t id, uint16_t interval_ms)
{
    if (id < MAX_TASKS)
    {
        tasks[id].interval_ms = interval_ms;
        tasks[id].counter     = interval_ms;
    }
}

/* call in main while(1) */
void task_sched_run(void)
{
    if (!sched_tick) return;
    sched_tick = 0;

    for (uint8_t i = 0; i < task_count; i++)
    {
        if (!tasks[i].enable || tasks[i].func == ((void *)0)) continue;

        if (--tasks[i].counter == 0)
        {
            tasks[i].counter = tasks[i].interval_ms;
            tasks[i].func();
        }
    }
}
