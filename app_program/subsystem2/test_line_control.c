/**
 * @file test_line_control.c
 * @brief Test the pure line-following recovery policy.
 */
#include "test_line_control.h"

#if APP_TEST

#include "line_control.h"
#include "subsystem2.h"

bool
test_line_control_crossing (void)
{
    line_control_cfg_t const cfg = {100u, 200u, 300u};
    line_control_t control;
    bool b_passed;

    b_passed = line_control_init(&control, &cfg, 0u);
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_CENTER, 0u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_BOTH_BLACK, 10u))
        && (LINE_CONTROL_MODE_CROSSING == line_control_mode(&control));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_BOTH_BLACK, 109u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STOP
            == line_control_step(&control, S2_LINE_BOTH_BLACK, 110u));

    b_passed = b_passed && line_control_init(&control, &cfg, 0u);
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_BOTH_BLACK, 10u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CORRECT_RIGHT
            == line_control_step(&control, S2_LINE_RIGHT, 50u))
        && (LINE_CONTROL_MODE_FOLLOW == line_control_mode(&control));

    return (b_passed);
}

bool
test_line_control_search (void)
{
    line_control_cfg_t const cfg = {100u, 200u, 300u};
    line_control_cfg_t const invalid_cfg = {0u, 200u, 300u};
    line_control_t control;
    bool b_passed;

    b_passed = line_control_init(&control, &cfg, 0u);
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CORRECT_LEFT
            == line_control_step(&control, S2_LINE_LEFT, 0u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CORRECT_LEFT
            == line_control_step(&control, S2_LINE_LEFT, 199u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_PIVOT_LEFT
            == line_control_step(&control, S2_LINE_LEFT, 200u))
        && (LINE_CONTROL_MODE_SEARCH == line_control_mode(&control));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_CENTER, 250u))
        && (LINE_CONTROL_MODE_FOLLOW == line_control_mode(&control));

    b_passed = b_passed && line_control_init(&control, &cfg, UINT32_MAX - 50u);
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CORRECT_RIGHT
            == line_control_step(&control, S2_LINE_RIGHT,
                                 UINT32_MAX - 50u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_PIVOT_RIGHT
            == line_control_step(&control, S2_LINE_RIGHT, 149u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_PIVOT_RIGHT
            == line_control_step(&control, S2_LINE_RIGHT, 448u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STOP
            == line_control_step(&control, S2_LINE_RIGHT, 449u));

    b_passed = b_passed
        && (!line_control_init(&control, &invalid_cfg, 0u))
        && (LINE_CONTROL_MODE_INVALID == line_control_mode(&control))
        && (LINE_CONTROL_ACTION_STOP
            == line_control_step(&control, S2_LINE_CENTER, 0u))
        && (!line_control_init(NULL, &cfg, 0u))
        && (!line_control_init(&control, NULL, 0u));

    return (b_passed);
}

#endif /* APP_TEST */

/* End of file. */
