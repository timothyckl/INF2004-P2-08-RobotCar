/**
 * @file test_scan_control.c
 * @brief Test the pure Scan & Plan handshake (scan_control.c).
 */
#include "test_scan_control.h"

#if APP_TEST

#include <stdint.h>

#include "scan_control.h"
#include "subsystem2.h"

#define TEST_SCAN_LIMIT_MS  20000u

static bool test_scan_msg(scan_control_output_t const * p_out, uint8_t index,
                          uint8_t kind);

/* Begin stops the wheels and sends REQUEST once, with a nonzero ID. */
bool
test_scan_control_begin (void)
{
    scan_control_t        scan;
    scan_control_output_t out;
    uint32_t              first_id;
    bool                  b_passed;

    b_passed = scan_control_init(&scan, TEST_SCAN_LIMIT_MS)
               && !scan_control_init(&scan, 0u)
               && !scan_control_active(&scan);

    scan_control_begin(&scan, 100u, &out);
    first_id = scan.request_id;
    b_passed = b_passed && out.b_stop_motion && (1u == out.n_msg)
               && test_scan_msg(&out, 0u, S2_SCAN_REQUEST)
               && (0u != first_id) && scan_control_active(&scan);

    /* A second hazard while scanning starts nothing new. */
    scan_control_begin(&scan, 150u, &out);
    b_passed = b_passed && (!out.b_stop_motion) && (0u == out.n_msg)
               && (first_id == scan.request_id);

    return (b_passed);
}

/* STOP_CONFIRM and AUTHORISE only once both wheels are still. */
bool
test_scan_control_grant (void)
{
    scan_control_t        scan;
    scan_control_output_t out;
    bool                  b_passed;

    b_passed = scan_control_init(&scan, TEST_SCAN_LIMIT_MS);
    scan_control_begin(&scan, 0u, &out);

    scan_control_step(&scan, false, true, scan.request_id,
                      SCAN_CONTROL_RESULT_NONE, 50u, &out);
    b_passed = b_passed && (0u == out.n_msg)
               && (SCAN_CONTROL_STOPPING == scan.state);

    scan_control_step(&scan, true, true, scan.request_id,
                      SCAN_CONTROL_RESULT_NONE, 150u, &out);
    b_passed = b_passed && (2u == out.n_msg)
               && test_scan_msg(&out, 0u, S2_SCAN_STOP_CONFIRM)
               && test_scan_msg(&out, 1u, S2_SCAN_AUTHORISE)
               && (SCAN_CONTROL_SCANNING == scan.state);

    scan_control_step(&scan, true, true, scan.request_id,
                      SCAN_CONTROL_RESULT_NONE, 200u, &out);
    b_passed = b_passed && (0u == out.n_msg)
               && (SCAN_CONTROL_NONE == out.outcome);

    return (b_passed);
}

/* Results for this request end the scan; other IDs are ignored. */
bool
test_scan_control_outcomes (void)
{
    scan_control_t        scan;
    scan_control_output_t out;
    bool                  b_passed;

    b_passed = scan_control_init(&scan, TEST_SCAN_LIMIT_MS);
    scan_control_begin(&scan, 0u, &out);
    scan_control_step(&scan, true, true, scan.request_id,
                      SCAN_CONTROL_RESULT_NONE, 10u, &out);

    /* A PLAN for an older request is not this scan's result. */
    scan_control_step(&scan, true, true, scan.request_id + 1u,
                      SCAN_CONTROL_RESULT_PLAN, 20u, &out);
    b_passed = b_passed && (SCAN_CONTROL_NONE == out.outcome)
               && scan_control_active(&scan);

    scan_control_step(&scan, true, true, scan.request_id,
                      SCAN_CONTROL_RESULT_PLAN, 30u, &out);
    b_passed = b_passed && (SCAN_CONTROL_PLAN == out.outcome)
               && !scan_control_active(&scan);

    scan_control_begin(&scan, 100u, &out);
    scan_control_step(&scan, true, true, scan.request_id,
                      SCAN_CONTROL_RESULT_CLEAR, 110u, &out);
    b_passed = b_passed && (SCAN_CONTROL_CLEAR == out.outcome);

    /* No route (3), fault (4) and cancelled (5) all fail the scan. */
    scan_control_begin(&scan, 200u, &out);
    scan_control_step(&scan, true, true, scan.request_id, 3u, 210u, &out);
    b_passed = b_passed && (SCAN_CONTROL_FAILED == out.outcome);

    /* A result before the wheels are still is not accepted. */
    scan_control_begin(&scan, 300u, &out);
    scan_control_step(&scan, false, true, scan.request_id,
                      SCAN_CONTROL_RESULT_PLAN, 310u, &out);
    b_passed = b_passed && (SCAN_CONTROL_NONE == out.outcome)
               && scan_control_active(&scan);

    return (b_passed);
}

/* Timeout sends CANCEL; abort sends STOP; idle abort sends nothing. */
bool
test_scan_control_end (void)
{
    scan_control_t        scan;
    scan_control_output_t out;
    bool                  b_passed;

    b_passed = scan_control_init(&scan, TEST_SCAN_LIMIT_MS);
    scan_control_begin(&scan, 1000u, &out);
    scan_control_step(&scan, true, true, scan.request_id,
                      SCAN_CONTROL_RESULT_NONE, 1000u + TEST_SCAN_LIMIT_MS,
                      &out);
    b_passed = b_passed && (SCAN_CONTROL_NONE == out.outcome);

    scan_control_step(&scan, true, true, scan.request_id,
                      SCAN_CONTROL_RESULT_NONE,
                      1001u + TEST_SCAN_LIMIT_MS, &out);
    b_passed = b_passed && (1u == out.n_msg)
               && test_scan_msg(&out, 0u, S2_SCAN_CANCEL)
               && (SCAN_CONTROL_TIMEOUT == out.outcome)
               && !scan_control_active(&scan);

    scan_control_begin(&scan, 50000u, &out);
    scan_control_abort(&scan, &out);
    b_passed = b_passed && (1u == out.n_msg)
               && test_scan_msg(&out, 0u, S2_SCAN_STOP)
               && (SCAN_CONTROL_FAILED == out.outcome)
               && !scan_control_active(&scan);

    scan_control_abort(&scan, &out);
    b_passed = b_passed && (0u == out.n_msg)
               && (SCAN_CONTROL_NONE == out.outcome);

    return (b_passed);
}

static bool
test_scan_msg (scan_control_output_t const * p_out, uint8_t index,
               uint8_t kind)
{
    return ((index < p_out->n_msg) && (kind == p_out->msg[index].kind)
            && p_out->msg[index].b_asserted);
}

#endif /* APP_TEST */

/* End of file. */
