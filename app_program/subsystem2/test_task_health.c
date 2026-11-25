/**
 * @file test_task_health.c
 * @brief Test the task-liveness evaluator (task_health.c, S2 decision 37).
 */
#include "test_task_health.h"

#if APP_TEST

#include <stdint.h>

#include "task_health.h"
#include "../app_stack.h"

/*
 * S2-U81 (post-demo, 2026-11-25): s2_health_evaluate() on synthetic
 * records. A task never seen is not monitored; a seen task is stalled
 * only when its last check-in is more than its limit ago (500 ms; the S3
 * loop 1000 ms; the IMU 1500 ms): exactly the limit is not a stall, 1 ms
 * more is; the result is one bit per task; differences are taken across
 * the 32-bit clock wrap; NULL records give 0. Added 2026-11-26: a
 * check-in stamped after now_ms is fresh, not a stall. Motion Control
 * reads the clock before the records, so a higher-priority task (the S3
 * loop at priority 1, the controller at 14) can preempt it in between
 * and check in later than now_ms; before the fix that read as a check-in
 * 2^32 ms old and latched a false MOTION_FAULT_TASK.
 */
bool
test_task_health_evaluate (void)
{
    s2_health_record_t records[S2_HEALTH_TASKS] =
    {
        {1000u, true},          /* controller */
        {1000u, true},          /* S3 line loop */
        {0u, false},            /* S5 scan task: never seen */
        {1000u, true}           /* S4 IMU task */
    };
    uint8_t const line_bit = (uint8_t)(1u << S2_HEALTH_LINE);
    uint8_t const ctrl_bit = (uint8_t)(1u << S2_HEALTH_CONTROLLER);
    uint8_t const imu_bit = (uint8_t)(1u << S2_HEALTH_IMU);
    bool b_ok;

    /* At the limit nothing is stalled; 1 ms later the controller is, but
       not the line loop or the IMU, and never the task not yet seen. */
    b_ok = (0u == s2_health_evaluate(records, 1000u + S2_HEALTH_LIMIT_MS))
           && (ctrl_bit
               == s2_health_evaluate(records,
                                     1000u + S2_HEALTH_LIMIT_MS + 1u))
           && (ctrl_bit
               == s2_health_evaluate(records,
                                     1000u + S2_HEALTH_LINE_LIMIT_MS))
           && ((ctrl_bit | line_bit)
               == s2_health_evaluate(records,
                                     1000u + S2_HEALTH_LINE_LIMIT_MS + 1u));

    /* The IMU at its own limit, then past it. */
    b_ok = b_ok
           && (0u == (imu_bit
                      & s2_health_evaluate(records,
                                           1000u + S2_HEALTH_IMU_LIMIT_MS)))
           && (0u != (imu_bit
                      & s2_health_evaluate(records,
                                           1000u + S2_HEALTH_IMU_LIMIT_MS
                                           + 1u)));

    /* The controller checks in again: only the line loop stays stalled. */
    records[S2_HEALTH_CONTROLLER].last_ms = 1900u;
    b_ok = b_ok
           && (line_bit == s2_health_evaluate(records,
                                              1000u + S2_HEALTH_LINE_LIMIT_MS
                                              + 1u));

    /* Across the wrap: 100 ms before it, then 400 ms after. */
    records[S2_HEALTH_CONTROLLER].last_ms = UINT32_MAX - 99u;
    records[S2_HEALTH_LINE].b_seen = false;
    records[S2_HEALTH_IMU].b_seen = false;
    b_ok = b_ok
           && (0u == s2_health_evaluate(records, S2_HEALTH_LIMIT_MS - 100u))
           && (ctrl_bit
               == s2_health_evaluate(records, S2_HEALTH_LIMIT_MS - 99u));

    /* Checked in after the guard read its clock: 10 ms (one tick) and
       1 ms later are fresh, also across the wrap. The oldest age taken as
       a stall is half the clock range. */
    records[S2_HEALTH_CONTROLLER].last_ms = 5010u;
    b_ok = b_ok && (0u == s2_health_evaluate(records, 5000u));
    records[S2_HEALTH_CONTROLLER].last_ms = 0u;
    b_ok = b_ok && (0u == s2_health_evaluate(records, UINT32_MAX));
    records[S2_HEALTH_CONTROLLER].last_ms = 1u;
    b_ok = b_ok
           && (ctrl_bit == s2_health_evaluate(records, 0x80000000u))
           && (0u == s2_health_evaluate(records, 0x80000001u));

    return (b_ok && (0u == s2_health_evaluate(NULL, 0u)));
}

/*
 * S2-U83 (post-demo, 2026-11-26, review F-24): the stack high-water scan
 * (app_stack.c). A painted stack is wholly unused; a task that wrote the
 * top 100 words leaves the rest; one write near the bottom marks all
 * above it as used, as a deep call would; a fully used stack is 0; NULL
 * is 0 and painting NULL does nothing.
 */
bool
test_app_stack_high_water (void)
{
    uint32_t words[544];        /* 2176 bytes, one pooled stack */
    bool b_ok;

    app_stack_paint(words, 544u);
    b_ok = (2176u == app_stack_unused_bytes(words, 544u));

    for (uint32_t idx = 444u; idx < 544u; idx++)
    {
        words[idx] = idx;       /* the top 100 words used */
    }
    b_ok = b_ok && ((444u * 4u) == app_stack_unused_bytes(words, 544u));

    words[10] = 0u;             /* a deep call reached word 10 */
    b_ok = b_ok && (40u == app_stack_unused_bytes(words, 544u));

    words[0] = 0u;
    b_ok = b_ok && (0u == app_stack_unused_bytes(words, 544u))
           && (0u == app_stack_unused_bytes(NULL, 544u));
    app_stack_paint(NULL, 544u);

    return (b_ok);
}

#endif /* APP_TEST */

/* End of file. */
