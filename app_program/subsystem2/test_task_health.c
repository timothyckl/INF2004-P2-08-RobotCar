/**
 * @file test_task_health.c
 * @brief Test the task-liveness evaluator (task_health.c, S2 decision 37).
 */
#include "test_task_health.h"

#if APP_TEST

#include <stdint.h>

#include "task_health.h"

/*
 * S2-U81 (post-demo, 2026-11-25): s2_health_evaluate() on synthetic
 * records. A task never seen is not monitored; a seen task is stalled
 * only when its last check-in is more than its limit ago (500 ms; the S3
 * loop 1000 ms; the IMU 1500 ms): exactly the limit is not a stall, 1 ms
 * more is; the result is one bit per task; differences are taken across
 * the 32-bit clock wrap; NULL records give 0.
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

    return (b_ok && (0u == s2_health_evaluate(NULL, 0u)));
}

#endif /* APP_TEST */

/* End of file. */
