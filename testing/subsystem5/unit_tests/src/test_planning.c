/**
 * @file test_planning.c
 * @brief Forward monitoring (decision 3) and bypass planning (decision 5).
 *
 * Profiles and geometry are synthetic fixtures, not robot measurements.
 * Profile index 0 is -60 degrees (right), index 8 is +60 degrees (left).
 */
#include "../include/test_planning.h"
#include "../include/test_runner.h"
#include "../../../../app_program/subsystem5/include/bypass_plan.h"
#include "../../../../app_program/subsystem5/include/forward_monitor.h"

#include <stddef.h>

/* Synthetic geometry: 80 mm half width, 50 mm clearance, 20 mm margin,
   200 mm depth, 500 mm search bound, 4000 mm range. */
static bypass_plan_config_t const g_planning_config =
{
    80u, 50u, 20u, 200u, 500u, 4000u
};

/** S5-U25: unsafe, invalid and stale readings request a stop. */
static bool test_planning_monitor (test_interface_api_t const * p_api);

/** S5-U36: the shorter route wins; left wins a tie. */
static bool test_planning_shorter (test_interface_api_t const * p_api);

/** S5-U37: blocked or unobserved sides give no usable plan. */
static bool test_planning_no_route (test_interface_api_t const * p_api);

/** S5-U53: plan segments, search bound and alignment. */
static bool test_planning_segments (test_interface_api_t const * p_api);

/** S5-U54: nothing in range ahead is not an obstacle. */
static bool test_planning_clear (test_interface_api_t const * p_api);

void
test_planning_run (test_interface_api_t const * p_api)
{
    static test_runner_case_t const cases[] =
    {
        {"S5-U25", "unsafe, invalid or stale data stops",
         test_planning_monitor},
        {"S5-U36", "shorter route wins, left on a tie",
         test_planning_shorter},
        {"S5-U37", "blocked or unobserved sides give no plan",
         test_planning_no_route},
        {"S5-U53", "plan segments, search bound and align",
         test_planning_segments},
        {"S5-U54", "nothing ahead in range is no obstacle",
         test_planning_clear}
    };
    uint32_t const count = (uint32_t) (sizeof(cases) / sizeof(cases[0]));

    test_runner_cases(cases, count, p_api);
}

static bool
test_planning_monitor (test_interface_api_t const * p_api)
{
    forward_monitor_config_t const config = {250u, 150000u};
    forward_monitor_t              monitor;

    (void) p_api;
    forward_monitor_reset(&monitor);

    bool const b_stale_start = (FORWARD_MONITOR_STALE
                                == forward_monitor_check(&monitor, &config,
                                                         0u));

    forward_monitor_sample(&monitor, true, 600u, 1000u);

    bool const b_safe = (FORWARD_MONITOR_SAFE
                         == forward_monitor_check(&monitor, &config,
                                                  151000u));
    bool const b_stale = (FORWARD_MONITOR_STALE
                          == forward_monitor_check(&monitor, &config,
                                                   151001u));

    forward_monitor_sample(&monitor, true, 249u, 200000u);

    bool const b_unsafe = (FORWARD_MONITOR_UNSAFE
                           == forward_monitor_check(&monitor, &config,
                                                    200000u));

    forward_monitor_sample(&monitor, true, 250u, 210000u);

    bool const b_boundary = (FORWARD_MONITOR_SAFE
                             == forward_monitor_check(&monitor, &config,
                                                      210000u));

    forward_monitor_sample(&monitor, false, 3000u, 220000u);

    bool const b_invalid = (FORWARD_MONITOR_INVALID
                            == forward_monitor_check(&monitor, &config,
                                                     220000u));

    /* Freshness across the 32-bit wrap. */
    forward_monitor_sample(&monitor, true, 900u, UINT32_MAX - 1000u);

    bool const b_wrap = (FORWARD_MONITOR_SAFE
                         == forward_monitor_check(&monitor, &config, 5000u));

    return (b_stale_start && b_safe && b_stale && b_unsafe && b_boundary
            && b_invalid && b_wrap);
}

static bool
test_planning_shorter (test_interface_api_t const * p_api)
{
    /* Obstacle 400 mm ahead, wider to the left: right is shorter. */
    uint16_t const wide_left[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 420u, 410u, 400u, 410u, 420u, 450u, 3000u
    };
    uint16_t const even[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 3000u, 410u, 400u, 410u, 3000u, 3000u, 3000u
    };
    bypass_plan_t plan;

    (void) p_api;

    bool const b_right = bypass_plan_make(wide_left, &g_planning_config,
                                          &plan)
                         && (BYPASS_PLAN_RIGHT == plan.side);
    bool const b_tie   = bypass_plan_make(even, &g_planning_config, &plan)
                         && (BYPASS_PLAN_LEFT == plan.side);

    return (b_right && b_tie);
}

static bool
test_planning_no_route (test_interface_api_t const * p_api)
{
    /* Wall: no beam passes on either side within 60 degrees. */
    uint16_t const wall[BYPASS_PLAN_ANGLES] =
    {
        460u, 450u, 430u, 410u, 400u, 410u, 430u, 450u, 460u
    };
    /* Edges observed, but too wide to return within 500 mm. */
    uint16_t const wide[BYPASS_PLAN_ANGLES] =
    {
        3000u, 590u, 560u, 540u, 520u, 540u, 560u, 590u, 3000u
    };
    uint16_t       missing[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 3000u, 410u, 400u, 410u, 3000u, 3000u, 3000u
    };
    bypass_plan_t plan;

    (void) p_api;

    bool const b_wall = (!bypass_plan_make(wall, &g_planning_config, &plan))
                        && (BYPASS_PLAN_NO_ROUTE == plan.result)
                        && (0u == plan.count);
    bool const b_wide = (!bypass_plan_make(wide, &g_planning_config, &plan))
                        && (BYPASS_PLAN_NO_ROUTE == plan.result);

    missing[6] = 0u;    /* an invalid (unobserved) sample */

    bool const b_invalid =
        (!bypass_plan_make(missing, &g_planning_config, &plan))
        && (BYPASS_PLAN_INVALID == plan.result)
        && (!bypass_plan_make(NULL, &g_planning_config, &plan))
        && (!bypass_plan_make(wall, NULL, &plan));

    return (b_wall && b_wide && b_invalid);
}

static bool
test_planning_segments (test_interface_api_t const * p_api)
{
    /* Right edge hit at -15 degrees and 410 mm: 410 x 0.259 = 106 mm. */
    uint16_t const profile[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 3000u, 410u, 400u, 420u, 430u, 3000u, 3000u
    };
    bypass_plan_t plan;

    (void) p_api;

    bool const b_made = bypass_plan_make(profile, &g_planning_config, &plan);
    uint32_t const lateral = 106u + 80u + 50u + 20u;

    return (b_made && (BYPASS_PLAN_RIGHT == plan.side) && (4u == plan.count)
            && (BYPASS_PLAN_TURN == plan.segments[0].kind)
            && (-90000 == plan.segments[0].value)
            && (BYPASS_PLAN_MOVE == plan.segments[1].kind)
            && ((int32_t) lateral == plan.segments[1].value)
            && (90000 == plan.segments[2].value)
            && ((400 + 200 + 50 + 20) == plan.segments[3].value)
            && (90000 == plan.approach_mdeg)
            && (-90000 == plan.align_mdeg)
            && ((lateral + 20u) == plan.search_mm)
            && (plan.search_mm <= g_planning_config.search_max_mm));
}

static bool
test_planning_clear (test_interface_api_t const * p_api)
{
    uint16_t const open[BYPASS_PLAN_ANGLES] =
    {
        4000u, 4000u, 4000u, 4000u, 4000u, 4000u, 4000u, 4000u, 4000u
    };
    bypass_plan_t plan;

    (void) p_api;

    return ((!bypass_plan_make(open, &g_planning_config, &plan))
            && (BYPASS_PLAN_NO_OBSTACLE == plan.result));
}

/* End of file. */
