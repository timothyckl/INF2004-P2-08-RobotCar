/**
 * @file task_health.c
 * @brief Liveness of the safety-critical tasks (task_health.h).
 */
#include "task_health.h"

/* No <stddef.h>: tk/syslib.h (through motion_api.h) defines size_t and
   NULL itself, and the two definitions conflict on the target. */
#include "motion_api.h"

/* One writer each (the task named by the index); read by Motion Control. */
static volatile uint32_t g_health_last_ms[S2_HEALTH_TASKS];
static volatile bool     g_health_seen[S2_HEALTH_TASKS];

static uint32_t const g_health_limit_ms[S2_HEALTH_TASKS] =
{
    S2_HEALTH_LIMIT_MS,         /* controller */
    S2_HEALTH_LINE_LIMIT_MS,    /* S3 line loop */
    S2_HEALTH_LIMIT_MS,         /* S5 scan task */
    S2_HEALTH_IMU_LIMIT_MS      /* S4 IMU task */
};

void
s2_health_check_in (s2_health_task_t task)
{
    SYSTIM now = {0};

    /* task and S2_HEALTH_TASKS are 0..4, which uint32_t holds unchanged. */
    if ((uint32_t)task < (uint32_t)S2_HEALTH_TASKS)
    {
        (void)tk_get_tim(&now);
        /* The time first, then "seen" (both volatile, so in this order).
           now.lo is the 32-bit ms word (UW): same width, unchanged. */
        g_health_last_ms[task] = (uint32_t)now.lo;
        g_health_seen[task] = true;
    }
}

void
s2_health_read (s2_health_record_t p_records[S2_HEALTH_TASKS])
{
    if (NULL != p_records)
    {
        /* S2_HEALTH_TASKS is 4, which uint32_t holds unchanged. */
        for (uint32_t task = 0u; task < (uint32_t)S2_HEALTH_TASKS; task++)
        {
            /* "seen" first: a time is used only once it was written. */
            p_records[task].b_seen = g_health_seen[task];
            p_records[task].last_ms = g_health_last_ms[task];
        }
    }
}

uint8_t
s2_health_evaluate (s2_health_record_t const p_records[S2_HEALTH_TASKS],
                    uint32_t now_ms)
{
    uint8_t stalled = 0u;

    if (NULL != p_records)
    {
        /* S2_HEALTH_TASKS is 4, which uint32_t holds unchanged. */
        for (uint32_t task = 0u; task < (uint32_t)S2_HEALTH_TASKS; task++)
        {
            /* Modulo 2^32 difference: correct across the clock wrap. */
            if (p_records[task].b_seen
                && ((now_ms - p_records[task].last_ms)
                    > g_health_limit_ms[task]))
            {
                /* task < S2_HEALTH_TASKS (4), so the bit fits uint8_t. */
                stalled = (uint8_t)(stalled | (1u << task));
            }
        }
    }

    return (stalled);
}

uint8_t
s2_health_apply (s2_health_record_t const p_records[S2_HEALTH_TASKS],
                 uint32_t now_ms, motion_t * p_motion,
                 motion_output_t * p_out)
{
    uint8_t const stalled = s2_health_evaluate(p_records, now_ms);

    if ((0u != stalled) && (NULL != p_motion) && (NULL != p_out))
    {
        /* Does nothing if already in Fault, so the first cause stays. */
        motion_fault(p_motion, MOTION_FAULT_TASK, p_out);
    }

    return (stalled);
}

uint8_t
s2_health_guard (motion_t * p_motion, motion_output_t * p_out,
                 uint32_t now_ms)
{
    s2_health_record_t records[S2_HEALTH_TASKS];

    s2_health_read(records);

    return (s2_health_apply(records, now_ms, p_motion, p_out));
}

char const *
s2_health_name (s2_health_task_t task)
{
    static char const * const names[S2_HEALTH_TASKS + 1] =
    {
        "controller", "S3 line loop", "S5 scan task", "S4 IMU task", "?"
    };

    /* task and S2_HEALTH_TASKS are 0..4, which uint32_t holds. */
    return (((uint32_t)task < (uint32_t)S2_HEALTH_TASKS)
            ? names[task] : names[S2_HEALTH_TASKS]);
}

/* End of file. */
