/**
 * @file test_look.c
 * @brief Stationary side look during a bypass (S2 decision 19).
 *
 * Times and readings are synthetic fixtures, not robot measurements.
 */
#include "../include/test_look.h"
#include "../include/test_runner.h"
#include "../../../../app_program/subsystem5/include/scan_look.h"

#include <stddef.h>

/* 300 ms settling, 1 s to stop, 2 s for the whole look. */
static scan_look_cfg_t const g_look_config = {300000u, 1000000u, 2000000u};

/** S5-U68: still, servo out, median of three, servo back, fresh reading. */
static bool test_look_sequence (test_interface_api_t const * p_api);

/** S5-U69: readings outside the sampling windows do not count. */
static bool test_look_windows (test_interface_api_t const * p_api);

/** S5-U70: invalid readings, movement, authority and time limits fault. */
static bool test_look_faults (test_interface_api_t const * p_api);

/** Start a look and bring it to SAMPLE at 300 ms with the wheels still. */
static void test_look_to_sample (scan_look_t * p_look,
                                 scan_look_output_t * p_out);

void
test_look_run (test_interface_api_t const * p_api)
{
    static test_runner_case_t const cases[] =
    {
        {"S5-U68", "side look sequence and median",
         test_look_sequence},
        {"S5-U69", "only windowed readings count",
         test_look_windows},
        {"S5-U70", "invalid, moved, authority, time faults",
         test_look_faults}
    };
    uint32_t const count = (uint32_t) (sizeof(cases) / sizeof(cases[0]));

    test_runner_cases(cases, count, p_api);
}

static void
test_look_to_sample (scan_look_t * p_look, scan_look_output_t * p_out)
{
    (void) scan_look_init(p_look, &g_look_config);
    scan_look_start(p_look, 7u, -45, 0u, p_out);
    scan_look_tick(p_look, 10000u, true, true, p_out);
    scan_look_tick(p_look, 310000u, true, true, p_out);
}

static bool
test_look_sequence (test_interface_api_t const * p_api)
{
    scan_look_t        look;
    scan_look_output_t out;
    bool               b_passed;

    (void) p_api;
    (void) scan_look_init(&look, &g_look_config);
    scan_look_start(&look, 7u, -45, 0u, &out);
    b_passed = scan_look_active(&look) && (!scan_look_capturing(&look))
               && (!out.b_servo) && (SCAN_LOOK_NONE == out.result);

    /* The servo turns only once the wheels are confirmed still. */
    scan_look_tick(&look, 5000u, false, true, &out);
    b_passed = b_passed && (!out.b_servo);
    scan_look_tick(&look, 10000u, true, true, &out);
    b_passed = b_passed && out.b_servo && (-45 == out.servo_deg)
               && (!scan_look_capturing(&look));

    /* Settling: sampling starts 300 ms after the servo command. */
    scan_look_tick(&look, 309999u, true, true, &out);
    b_passed = b_passed && (!scan_look_capturing(&look));
    scan_look_tick(&look, 310000u, true, true, &out);
    b_passed = b_passed && scan_look_capturing(&look);

    /* Median of 420, 380 and 400; then the servo turns forward. */
    scan_look_sample(&look, true, 420u, 375000u, &out);
    scan_look_sample(&look, true, 380u, 440000u, &out);
    b_passed = b_passed && (!out.b_servo);
    scan_look_sample(&look, true, 400u, 505000u, &out);
    b_passed = b_passed && out.b_servo && (0 == out.servo_deg)
               && (!scan_look_capturing(&look));

    /* Back and settled: forward monitoring restarts, then one fresh
       forward reading ends the look with the side median. */
    scan_look_tick(&look, 805000u, true, true, &out);
    b_passed = b_passed && out.b_resume && scan_look_capturing(&look);
    scan_look_sample(&look, true, 900u, 870000u, &out);

    return (b_passed && (SCAN_LOOK_DONE == out.result)
            && (400u == out.distance_mm) && (!scan_look_active(&look)));
}

static bool
test_look_windows (test_interface_api_t const * p_api)
{
    scan_look_t        look;
    scan_look_output_t out;
    bool               b_passed;

    (void) p_api;
    (void) scan_look_init(&look, &g_look_config);
    scan_look_start(&look, 8u, 45, 0u, &out);

    /* Readings while stopping or settling are ignored, even invalid. */
    scan_look_sample(&look, false, 0u, 1000u, &out);
    scan_look_sample(&look, false, 0u, 2000u, &out);
    scan_look_sample(&look, false, 0u, 3000u, &out);
    b_passed = scan_look_active(&look) && (SCAN_LOOK_NONE == out.result);
    scan_look_tick(&look, 10000u, true, true, &out);
    scan_look_sample(&look, true, 100u, 20000u, &out);
    scan_look_tick(&look, 310000u, true, true, &out);

    /* One invalid reading between valid ones is tolerated. */
    scan_look_sample(&look, true, 500u, 320000u, &out);
    scan_look_sample(&look, false, 0u, 385000u, &out);
    scan_look_sample(&look, true, 520u, 450000u, &out);
    scan_look_sample(&look, true, 510u, 515000u, &out);
    b_passed = b_passed && out.b_servo && (0 == out.servo_deg);

    /* Readings while the servo returns do not end the look. */
    scan_look_sample(&look, true, 900u, 580000u, &out);
    b_passed = b_passed && scan_look_active(&look);
    scan_look_tick(&look, 815000u, true, true, &out);
    scan_look_sample(&look, true, 900u, 880000u, &out);

    return (b_passed && (SCAN_LOOK_DONE == out.result)
            && (510u == out.distance_mm));
}

static bool
test_look_faults (test_interface_api_t const * p_api)
{
    scan_look_t        look;
    scan_look_output_t out;
    bool               b_passed;

    (void) p_api;

    /* Three consecutive invalid readings. */
    test_look_to_sample(&look, &out);
    scan_look_sample(&look, false, 0u, 320000u, &out);
    scan_look_sample(&look, false, 0u, 385000u, &out);
    b_passed = (SCAN_LOOK_NONE == out.result);
    scan_look_sample(&look, false, 0u, 450000u, &out);
    b_passed = b_passed && (SCAN_LOOK_FAULT == out.result) && out.b_servo
               && (0 == out.servo_deg) && out.b_resume
               && (!scan_look_active(&look));

    /* The wheels move after the servo turned. */
    test_look_to_sample(&look, &out);
    scan_look_tick(&look, 320000u, false, true, &out);
    b_passed = b_passed && (SCAN_LOOK_FAULT == out.result) && out.b_servo;

    /* Authority withdrawn (STOP, mode change). */
    test_look_to_sample(&look, &out);
    scan_look_tick(&look, 320000u, true, false, &out);
    b_passed = b_passed && (SCAN_LOOK_FAULT == out.result);

    /* The wheels never stop within 1 s. */
    (void) scan_look_init(&look, &g_look_config);
    scan_look_start(&look, 9u, 45, 0u, &out);
    scan_look_tick(&look, 1000000u, false, true, &out);
    b_passed = b_passed && (SCAN_LOOK_NONE == out.result);
    scan_look_tick(&look, 1000001u, false, true, &out);
    b_passed = b_passed && (SCAN_LOOK_FAULT == out.result);

    /* No reading within the 2 s limit. */
    test_look_to_sample(&look, &out);
    scan_look_tick(&look, 2000001u, true, true, &out);
    b_passed = b_passed && (SCAN_LOOK_FAULT == out.result);

    /* An angle beyond the servo's limit faults at once. */
    (void) scan_look_init(&look, &g_look_config);
    scan_look_start(&look, 10u, 46, 0u, &out);
    b_passed = b_passed && (SCAN_LOOK_FAULT == out.result)
               && (!scan_look_active(&look));

    /* A start while a look is active changes nothing. */
    test_look_to_sample(&look, &out);
    scan_look_start(&look, 11u, 30, 320000u, &out);

    return (b_passed && (SCAN_LOOK_NONE == out.result)
            && (7u == look.id) && scan_look_capturing(&look));
}

/* End of file. */
