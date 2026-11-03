/**
 * @file test_odometry.c
 * @brief Host cases for s4_odometry: encoder distance into terrain logic.
 *
 * Statuses are synthetic. Built with testing/host (kernel stand-ins).
 */
#include <stdio.h>

#include "../../app_program/subsystem2/motion_cmd.h"
#include "../../app_program/subsystem4/s4_odometry.h"

/* Created by task_motion.c on the Pico W; unused by these cases. */
ID motion_q = 0;

static unsigned g_checks = 0u;
static unsigned g_failures = 0u;

static void
check (bool b_passed, char const * p_name)
{
    g_checks++;

    if (!b_passed)
    {
        g_failures++;
        (void) printf("FAIL: %s\n", p_name);
    }
}

static motion_status_t
status_at (uint32_t captured_ms, int32_t travel_mm, uint8_t run_id)
{
    motion_status_t st = {0};

    st.captured_ms = captured_ms;
    st.travel_mm = travel_mm;
    st.run_id = run_id;
    st.flags = MOTION_ST_VALID;

    return (st);
}

int
main (void)
{
    s4_odometry_t odo;
    s4_odometry_step_t step;
    motion_status_t st;

    s4_odometry_init(&odo);
    st = status_at(1000u, 100, 1u);
    s4_odometry_step(&odo, &st, 1005u, &step);
    check(!step.b_valid && !step.b_new_run, "first status gives no distance");

    st = status_at(1010u, 104, 1u);
    s4_odometry_step(&odo, &st, 1015u, &step);
    check(step.b_valid && (4u == step.delta_mm), "aligned forward distance");

    st = status_at(1020u, 109, 1u);
    s4_odometry_step(&odo, &st, 1041u, &step);
    check(!step.b_valid, "status older than 20 ms is not used");

    st = status_at(1050u, 112, 1u);
    s4_odometry_step(&odo, &st, 1040u, &step);
    check(!step.b_valid, "status from after the sample is not used");

    st = status_at(1060u, 115, 1u);
    s4_odometry_step(&odo, &st, 1060u, &step);
    check(step.b_valid && (3u == step.delta_mm),
          "valid again after a skipped sample");

    s4_odometry_step(&odo, NULL, 1070u, &step);
    check(!step.b_valid, "unreadable status gives no distance");
    st = status_at(1080u, 120, 1u);
    s4_odometry_step(&odo, &st, 1080u, &step);
    check(!step.b_valid, "first status after a gap gives no distance");

    st = status_at(1090u, 0, 2u);
    s4_odometry_step(&odo, &st, 1090u, &step);
    check(step.b_new_run && !step.b_valid, "run_id change starts a new run");
    st = status_at(1100u, 6, 2u);
    s4_odometry_step(&odo, &st, 1100u, &step);
    check(step.b_valid && (6u == step.delta_mm) && !step.b_new_run,
          "distance resumes in the new run");

    st = status_at(1110u, 2, 2u);
    s4_odometry_step(&odo, &st, 1110u, &step);
    check(!step.b_valid, "reversing gives no forward distance");

    st = status_at(1120u, 3, 2u);
    st.flags = 0u;
    s4_odometry_step(&odo, &st, 1120u, &step);
    check(!step.b_valid, "invalid encoder status gives no distance");

    s4_odometry_init(&odo);
    st = status_at(UINT32_MAX - 5u, 10, 3u);
    s4_odometry_step(&odo, &st, UINT32_MAX - 2u, &step);
    st = status_at(4u, 15, 3u);
    s4_odometry_step(&odo, &st, 8u, &step);
    check(step.b_valid && (5u == step.delta_mm), "alignment across the wrap");

    (void) printf("%u checks, %u failures\n", g_checks, g_failures);

    return ((0u == g_failures) ? 0 : 1);
}

/* End of file. */
