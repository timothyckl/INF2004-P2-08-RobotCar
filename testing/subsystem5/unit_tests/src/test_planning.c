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

/* Synthetic geometry: 82 mm left and 63 mm right of the centre line,
   50 mm clearance, 20 mm margin, 200 mm depth, 500 mm search bound,
   4000 mm range. */
static bypass_plan_config_t const g_planning_config =
{
    82u, 63u, 50u, 20u, 200u, 500u, 4000u
};

/** S5-U25: unsafe, invalid and stale readings request a stop. */
static bool test_planning_monitor (test_interface_api_t const * p_api);

/** Forward invalid attempts fault only on the third consecutive sample. */
static bool test_planning_monitor_fault (test_interface_api_t const * p_api);

/** S5-U36: left wins whenever valid; right only when left is not. */
static bool test_planning_shorter (test_interface_api_t const * p_api);

/** S5-U37: blocked or unobserved sides give no usable plan. */
static bool test_planning_no_route (test_interface_api_t const * p_api);

/** S5-U53: plan segments, search bound and alignment. */
static bool test_planning_segments (test_interface_api_t const * p_api);

/** S5-U54: nothing in range ahead is not an obstacle. */
static bool test_planning_clear (test_interface_api_t const * p_api);

/** S5-U60: refined edges; left preferred, right if left is invalid. */
static bool test_planning_fine_edges (test_interface_api_t const * p_api);

/** S5-U61: incomplete or inconsistent edge readings yield no route. */
static bool test_planning_fine_reject (test_interface_api_t const * p_api);

/** S5-U62: observed rays give an explicitly estimated shape and clearance. */
static bool test_planning_profile (test_interface_api_t const * p_api);

/** S5-U63: missing one edge leaves shape fields unknown. */
static bool test_planning_partial_profile (test_interface_api_t const * p_api);

void
test_planning_run (test_interface_api_t const * p_api)
{
    static test_runner_case_t const cases[] =
    {
        {"S5-U25", "unsafe, invalid or stale data stops",
         test_planning_monitor},
        {"S5-U55", "third invalid forward attempt faults",
         test_planning_monitor_fault},
        {"S5-U36", "left whenever valid, else right",
         test_planning_shorter},
        {"S5-U37", "blocked or unobserved sides give no plan",
         test_planning_no_route},
        {"S5-U53", "plan segments, search bound and align",
         test_planning_segments},
        {"S5-U54", "nothing ahead in range is no obstacle",
         test_planning_clear},
        {"S5-U60", "fine edges: left preferred, else right",
         test_planning_fine_edges},
        {"S5-U61", "missing or inconsistent edge has no route",
         test_planning_fine_reject},
        {"S5-U62", "observed span and plan clearance estimate",
         test_planning_profile},
        {"S5-U63", "missing edge keeps shape unknown",
         test_planning_partial_profile}
    };
    uint32_t const count = (uint32_t) (sizeof(cases) / sizeof(cases[0]));

    test_runner_cases(cases, count, p_api);
}

static bool
test_planning_monitor_fault (test_interface_api_t const * p_api)
{
    forward_monitor_t monitor;

    (void) p_api;
    forward_monitor_reset(&monitor);
    forward_monitor_sample(&monitor, false, 0u, 100u);
    forward_monitor_sample(&monitor, false, 0u, 200u);

    bool const b_below = (2u == monitor.invalid_count)
                         && (!forward_monitor_faulted(&monitor));

    forward_monitor_sample(&monitor, true, 300u, 300u);

    bool const b_recovered = (0u == monitor.invalid_count)
                             && (!forward_monitor_faulted(&monitor));

    forward_monitor_sample(&monitor, false, 0u, 400u);
    forward_monitor_sample(&monitor, false, 0u, 500u);
    forward_monitor_sample(&monitor, false, 0u, 600u);

    return (b_below && b_recovered && (3u == monitor.invalid_count)
            && forward_monitor_faulted(&monitor));
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
    /* Obstacle 400 mm ahead, wider to the left: left is longer (450 x
       0.707 = 318 mm edge, + 63 + 50 + 20) but still valid, so it wins. */
    uint16_t const wide_left[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 420u, 410u, 400u, 410u, 420u, 450u, 3000u
    };
    /* The left side never passes within 60 degrees: right is used. */
    uint16_t const left_wall[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 3000u, 410u, 400u, 410u, 420u, 450u, 460u
    };
    bypass_plan_t plan;

    (void) p_api;

    bool const b_left  = bypass_plan_make(wide_left, &g_planning_config,
                                          &plan)
                         && (BYPASS_PLAN_LEFT == plan.side)
                         && ((318u + 63u + 50u + 20u) == plan.lateral_mm);
    bool const b_right = bypass_plan_make(left_wall, &g_planning_config,
                                          &plan)
                         && (BYPASS_PLAN_RIGHT == plan.side);

    return (b_left && b_right);
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
    /* Right edge hit at -15 degrees and 410 mm: 410 x 0.259 = 106 mm.
       Left is blocked to 60 degrees, so the right route is used, and it
       must clear the vehicle's wider left side (82 mm). */
    uint16_t const profile[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 3000u, 410u, 400u, 420u, 430u, 450u, 460u
    };
    bypass_plan_t plan;

    (void) p_api;

    bool const b_made = bypass_plan_make(profile, &g_planning_config, &plan);
    uint32_t const lateral = 106u + 82u + 50u + 20u;

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

static bool
test_planning_fine_edges (test_interface_api_t const * p_api)
{
    uint16_t profile[BYPASS_PLAN_FINE_ANGLES] = {0u};
    uint16_t const coarse[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 3000u, 410u, 400u, 410u, 3000u, 3000u, 3000u
    };
    bypass_plan_t plan;

    (void) p_api;

    for (uint32_t index = 0u; index < BYPASS_PLAN_ANGLES; index++)
    {
        profile[index * 3u] = coarse[index];
    }

    profile[7] = 3000u;  /* -25 degrees: right beam passes the edge. */
    profile[8] = 410u;   /* -20 degrees: right beam hits the edge. */
    profile[16] = 410u;  /* +20 degrees: left beam hits the edge. */
    profile[17] = 410u;  /* +25 degrees: left edge reaches farther. */

    /* Left edge 410 x 0.423 = 173 mm, right 410 x 0.342 = 140 mm: left is
       longer but valid, so it wins, sized by the right side (63 mm). */
    bool const b_left = bypass_plan_make_fine(profile, &g_planning_config,
                                               &plan)
                        && (BYPASS_PLAN_LEFT == plan.side)
                        && ((173u + 63u + 50u + 20u) == plan.lateral_mm)
                        && (plan.search_mm <= g_planning_config.search_max_mm);

    profile[17] = 0u;     /* Left edge reading missing: right is used. */

    bool const b_right = bypass_plan_make_fine(profile, &g_planning_config,
                                                &plan)
                         && (BYPASS_PLAN_RIGHT == plan.side)
                         && ((140u + 82u + 50u + 20u) == plan.lateral_mm);

    return (b_left && b_right);
}

static bool
test_planning_fine_reject (test_interface_api_t const * p_api)
{
    uint16_t profile[BYPASS_PLAN_FINE_ANGLES] = {0u};
    uint16_t const coarse[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 3000u, 410u, 400u, 410u, 3000u, 3000u, 3000u
    };
    bypass_plan_t plan;

    (void) p_api;

    for (uint32_t index = 0u; index < BYPASS_PLAN_ANGLES; index++)
    {
        profile[index * 3u] = coarse[index];
    }

    bool const b_missing = (!bypass_plan_make_fine(profile,
                                                    &g_planning_config,
                                                    &plan))
                           && (BYPASS_PLAN_NO_ROUTE == plan.result);

    profile[7] = 410u;   /* A far blocked ray after a nearer open ray. */
    profile[8] = 3000u;

    bool const b_inconsistent = (!bypass_plan_make_fine(profile,
                                                         &g_planning_config,
                                                         &plan))
                                && (BYPASS_PLAN_NO_ROUTE == plan.result);

    profile[7] = 3000u;
    profile[8] = 410u;
    profile[3] = 410u;   /* Another obstacle beyond the first right gap. */

    bool const b_second_obstacle = (!bypass_plan_make_fine(
                                      profile, &g_planning_config, &plan))
                                   && (BYPASS_PLAN_NO_ROUTE == plan.result);

    profile[12] = 0u;    /* Missing coarse forward measurement is invalid. */

    bool const b_invalid = (!bypass_plan_make_fine(profile,
                                                    &g_planning_config,
                                                    &plan))
                           && (BYPASS_PLAN_INVALID == plan.result);

    return (b_missing && b_inconsistent && b_second_obstacle && b_invalid);
}

static bool
test_planning_profile (test_interface_api_t const * p_api)
{
    uint16_t profile[BYPASS_PLAN_FINE_ANGLES] = {0u};
    uint16_t const coarse[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 3000u, 410u, 400u, 410u, 3000u, 3000u, 3000u
    };
    bypass_plan_t plan;

    (void) p_api;

    for (uint32_t index = 0u; index < BYPASS_PLAN_ANGLES; index++)
    {
        profile[index * 3u] = coarse[index];
    }

    profile[7] = 3000u;
    profile[8] = 410u;
    profile[16] = 410u;
    profile[17] = 410u;

    bool const b_plan = bypass_plan_make_fine(profile, &g_planning_config,
                                               &plan);

    return (b_plan && (BYPASS_PLAN_LEFT == plan.side)
            && (50u == plan.estimated_min_clearance_mm)
            && (400u == plan.profile.forward_mm)
            && (400u == plan.profile.nearest_observed_mm)
            && plan.profile.b_left_edge && plan.profile.b_right_edge
            && plan.profile.b_shape_estimated
            && (173u == plan.profile.left_extent_mm)
            && (140u == plan.profile.right_extent_mm)
            && (313u == plan.profile.observed_width_mm)
            && (16 == plan.profile.centre_offset_mm));
}

static bool
test_planning_partial_profile (test_interface_api_t const * p_api)
{
    uint16_t profile[BYPASS_PLAN_FINE_ANGLES] = {0u};
    uint16_t const coarse[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 3000u, 410u, 400u, 410u, 3000u, 3000u, 3000u
    };
    bypass_plan_t plan;

    (void) p_api;

    for (uint32_t index = 0u; index < BYPASS_PLAN_ANGLES; index++)
    {
        profile[index * 3u] = coarse[index];
    }

    profile[7] = 3000u;
    profile[8] = 410u;

    bool const b_plan = bypass_plan_make_fine(profile, &g_planning_config,
                                               &plan);
    bool const b_unknown_shape = (BYPASS_PLAN_RIGHT == plan.side)
                                 && plan.profile.b_right_edge
                                 && (!plan.profile.b_left_edge)
                                 && (!plan.profile.b_shape_estimated)
                                 && (0u == plan.profile.observed_width_mm)
                                 && (0 == plan.profile.centre_offset_mm);

    bypass_plan_config_t invalid_config = g_planning_config;

    invalid_config.range_max_mm = 4001u;

    bool const b_range_guard = (!bypass_plan_make_fine(profile,
                                                       &invalid_config, &plan))
                               && (BYPASS_PLAN_INVALID == plan.result);

    return (b_plan && b_unknown_shape && b_range_guard);
}

/* End of file. */
