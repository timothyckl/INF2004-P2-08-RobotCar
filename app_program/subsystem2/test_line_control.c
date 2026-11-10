/**
 * @file test_line_control.c
 * @brief Test the two-sensor line-following and junction policy.
 */
#include "test_line_control.h"

#if APP_TEST

#include <stdint.h>

#include "line_control.h"
#include "subsystem2.h"

static line_control_cfg_t const g_cfg =
{
    200u, 300u, 100u, 100u, 1500u, 600u, 100u, 150u, 2500u
};

bool
test_line_control_junction (void)
{
    line_control_t control;
    bool b_passed;

    b_passed = line_control_init(&control, &g_cfg, 0u);
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_CENTER, 0u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CORRECT_LEFT
            == line_control_step(&control, S2_LINE_LEFT, 10u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CURVE_LEFT
            == line_control_step(&control, S2_LINE_LEFT, 210u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CURVE_LEFT
            == line_control_step(&control, S2_LINE_BOTH_BLACK, 220u))
        && (LINE_CONTROL_MODE_JUNCTION_CONFIRM
            == line_control_mode(&control));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CURVE_LEFT
            == line_control_step(&control, S2_LINE_BOTH_BLACK, 319u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_BOTH_BLACK, 320u))
        && (LINE_CONTROL_MODE_CROSSING == line_control_mode(&control));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_CENTER, 330u))
        && (LINE_CONTROL_MODE_EXIT == line_control_mode(&control));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_LEFT, 929u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_LEFT, 930u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_CENTER, 940u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_CENTER, 1039u))
        && (LINE_CONTROL_MODE_EXIT == line_control_mode(&control));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_CENTER, 1040u))
        && (LINE_CONTROL_MODE_FOLLOW == line_control_mode(&control));

    b_passed = b_passed && line_control_init(&control, &g_cfg, 0u);
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_BOTH_BLACK, 0u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_BOTH_BLACK, 100u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_BOTH_BLACK, 1599u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STOP
            == line_control_step(&control, S2_LINE_BOTH_BLACK, 1600u));

    b_passed = b_passed && line_control_init(&control, &g_cfg, 0u);
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CORRECT_RIGHT
            == line_control_step(&control, S2_LINE_RIGHT, 0u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CORRECT_RIGHT
            == line_control_step(&control, S2_LINE_BOTH_BLACK, 10u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_CENTER, 109u))
        && (LINE_CONTROL_MODE_FOLLOW == line_control_mode(&control));

    b_passed = b_passed && line_control_init(&control, &g_cfg, 0u);
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_BOTH_BLACK, 0u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_BOTH_BLACK, 100u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_CENTER, 110u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STOP
            == line_control_step(&control, S2_LINE_CENTER, 2610u));

    b_passed = b_passed && line_control_init(&control, &g_cfg, 0u);
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_BOTH_BLACK, 0u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_BOTH_BLACK, 100u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_CENTER, 110u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_LEFT, 710u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_LEFT, 859u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CORRECT_LEFT
            == line_control_step(&control, S2_LINE_LEFT, 860u))
        && (LINE_CONTROL_MODE_FOLLOW == line_control_mode(&control));

    return (b_passed);
}

bool
test_line_control_curve (void)
{
    line_control_cfg_t const invalid_curve =
        {0u, 300u, 100u, 100u, 1500u, 600u, 100u, 150u, 2500u};
    line_control_cfg_t const invalid_hold =
        {200u, 0u, 100u, 100u, 1500u, 600u, 100u, 150u, 2500u};
    line_control_cfg_t const invalid_exit_order =
        {200u, 300u, 100u, 100u, 1500u, 600u, 100u, 150u, 600u};
    line_control_cfg_t const invalid_center =
        {200u, 300u, 100u, 100u, 1500u, 600u, 1901u, 150u, 2500u};
    line_control_cfg_t const invalid_side =
        {200u, 300u, 100u, 100u, 1500u, 600u, 100u, 0u, 2500u};
    line_control_t control;
    bool b_passed;

    b_passed = line_control_init(&control, &g_cfg, 0u);
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CORRECT_LEFT
            == line_control_step(&control, S2_LINE_LEFT, 0u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CORRECT_LEFT
            == line_control_step(&control, S2_LINE_LEFT, 199u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CURVE_LEFT
            == line_control_step(&control, S2_LINE_LEFT, 200u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CURVE_LEFT
            == line_control_step(&control, S2_LINE_CENTER, 499u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_CENTER, 500u));

    b_passed = b_passed && line_control_init(&control, &g_cfg, 0u);
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CORRECT_RIGHT
            == line_control_step(&control, S2_LINE_RIGHT, 0u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CURVE_RIGHT
            == line_control_step(&control, S2_LINE_RIGHT, 200u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CURVE_RIGHT
            == line_control_step(&control, S2_LINE_CENTER, 299u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_CENTER, 300u));

    b_passed = b_passed && line_control_init(&control, &g_cfg, 0u);
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CORRECT_LEFT
            == line_control_step(&control, S2_LINE_LEFT, 0u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CURVE_LEFT
            == line_control_step(&control, S2_LINE_LEFT, 200u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CORRECT_RIGHT
            == line_control_step(&control, S2_LINE_RIGHT, 210u));

    b_passed = b_passed
        && line_control_init(&control, &g_cfg, UINT32_MAX - 50u);
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CORRECT_RIGHT
            == line_control_step(&control, S2_LINE_RIGHT,
                                 UINT32_MAX - 50u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CORRECT_RIGHT
            == line_control_step(&control, S2_LINE_RIGHT, 148u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CURVE_RIGHT
            == line_control_step(&control, S2_LINE_RIGHT, 149u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CURVE_RIGHT
            == line_control_step(&control, S2_LINE_CENTER, 248u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_CENTER, 249u));

    b_passed = b_passed
        && (!line_control_init(&control, &invalid_curve, 0u))
        && (!line_control_init(&control, &invalid_hold, 0u))
        && (!line_control_init(&control, &invalid_exit_order, 0u))
        && (!line_control_init(&control, &invalid_center, 0u))
        && (!line_control_init(&control, &invalid_side, 0u))
        && (LINE_CONTROL_MODE_INVALID == line_control_mode(&control))
        && (LINE_CONTROL_ACTION_STOP
            == line_control_step(&control, S2_LINE_CENTER, 0u))
        && (!line_control_init(NULL, &g_cfg, 0u))
        && (!line_control_init(&control, NULL, 0u));

    return (b_passed);
}

#endif /* APP_TEST */

/* End of file. */
