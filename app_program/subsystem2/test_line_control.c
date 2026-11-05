/**
 * @file test_line_control.c
 * @brief Test the pure two-sensor line-following and recovery policy.
 */
#include "test_line_control.h"

#if APP_TEST

#include <stdint.h>

#include "line_control.h"
#include "subsystem2.h"

bool
test_line_control_lost (void)
{
    line_control_cfg_t const cfg = {200u, 300u, 150u, 100u};
    line_control_t control;
    bool b_passed;

    b_passed = line_control_init(&control, &cfg, 0u);
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_CENTER, 0u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STOP
            == line_control_step(&control, S2_LINE_LOST, 10u));

    b_passed = b_passed && line_control_init(&control, &cfg, 0u);
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CORRECT_LEFT
            == line_control_step(&control, S2_LINE_LEFT, 0u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_CENTER, 50u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CURVE_LEFT
            == line_control_step(&control, S2_LINE_LOST, 149u))
        && (LINE_CONTROL_MODE_LOST_RECOVERY
            == line_control_mode(&control));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CURVE_LEFT
            == line_control_step(&control, S2_LINE_LOST, 448u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STOP
            == line_control_step(&control, S2_LINE_LOST, 449u));

    b_passed = b_passed && line_control_init(&control, &cfg, 0u);
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CORRECT_RIGHT
            == line_control_step(&control, S2_LINE_RIGHT, 0u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CURVE_RIGHT
            == line_control_step(&control, S2_LINE_LOST, 10u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_CENTER, 50u))
        && (LINE_CONTROL_MODE_FOLLOW == line_control_mode(&control));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STOP
            == line_control_step(&control, S2_LINE_LOST, 151u));

    b_passed = b_passed && line_control_init(&control, &cfg, 0u);
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CORRECT_LEFT
            == line_control_step(&control, S2_LINE_LEFT, 0u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CURVE_LEFT
            == line_control_step(&control, S2_LINE_LOST, 10u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_RIGHT, 20u))
        && (LINE_CONTROL_MODE_FOLLOW == line_control_mode(&control));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CORRECT_RIGHT
            == line_control_step(&control, S2_LINE_RIGHT, 120u));

    return (b_passed);
}

bool
test_line_control_curve (void)
{
    line_control_cfg_t const cfg = {200u, 300u, 150u, 100u};
    line_control_cfg_t const invalid_curve = {0u, 300u, 150u, 100u};
    line_control_cfg_t const invalid_lost = {200u, 0u, 150u, 100u};
    line_control_cfg_t const invalid_memory = {200u, 300u, 0u, 100u};
    line_control_cfg_t const invalid_change = {200u, 300u, 150u, 0u};
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
        && (LINE_CONTROL_ACTION_CURVE_LEFT
            == line_control_step(&control, S2_LINE_LEFT, 200u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_CENTER, 201u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_RIGHT, 210u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_RIGHT, 309u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_CORRECT_RIGHT
            == line_control_step(&control, S2_LINE_RIGHT, 310u));

    b_passed = b_passed
        && line_control_init(&control, &cfg, UINT32_MAX - 50u);
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
            == line_control_step(&control, S2_LINE_LOST, 150u));
    b_passed = b_passed
        && (LINE_CONTROL_ACTION_STRAIGHT
            == line_control_step(&control, S2_LINE_CENTER, 151u));

    b_passed = b_passed
        && (!line_control_init(&control, &invalid_curve, 0u))
        && (!line_control_init(&control, &invalid_lost, 0u))
        && (!line_control_init(&control, &invalid_memory, 0u))
        && (!line_control_init(&control, &invalid_change, 0u))
        && (LINE_CONTROL_MODE_INVALID == line_control_mode(&control))
        && (LINE_CONTROL_ACTION_STOP
            == line_control_step(&control, S2_LINE_CENTER, 0u))
        && (!line_control_init(NULL, &cfg, 0u))
        && (!line_control_init(&control, NULL, 0u));

    return (b_passed);
}

#endif /* APP_TEST */

/* End of file. */
