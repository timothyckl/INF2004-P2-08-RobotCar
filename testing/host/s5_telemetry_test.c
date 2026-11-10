/**
 * @file s5_telemetry_test.c
 * @brief Host checks for S5's decoded, read-only S1 handoff.
 * Subsystem 5 -- Timothy.  Student ID: 2501530.
 */
#include "../../app_program/subsystem5/include/scan_task.h"
#include "../../app_program/subsystem5/include/scan_telemetry.h"

static scan_task_status_t g_snapshot;
static bool g_available;

ER
scan_task_get (scan_task_status_t * p_status)
{
    if ((!g_available) || (NULL == p_status))
    {
        return (E_OBJ);
    }

    *p_status = g_snapshot;

    return (E_OK);
}

int
main (void)
{
    scan_telemetry_t view = {0};
    scan_task_status_t const cleared = {0};

    g_snapshot = cleared;
    g_available = false;
    view.request_id = 99u;

    bool const b_busy = (!scan_telemetry_get(&view))
                        && (99u == view.request_id)
                        && (!scan_telemetry_get(NULL));

    g_available = true;

    bool const b_unready = (!scan_telemetry_get(&view))
                           && (99u == view.request_id);

    g_snapshot.captured_ms = 1234u;
    g_snapshot.request_id = 7u;
    g_snapshot.forward_mm = 400u;
    g_snapshot.forward_age_ms = 70u;
    g_snapshot.verdict = SCAN_TELEMETRY_SAFE;
    g_snapshot.flags = SCAN_TASK_FLAG_MONITORING;
    g_snapshot.result = SCAN_TASK_RESULT_PLAN;
    g_snapshot.plan.side = BYPASS_PLAN_LEFT;
    g_snapshot.plan.profile.forward_mm = 400u;
    g_snapshot.plan.profile.nearest_observed_mm = 380u;
    g_snapshot.plan.profile.observed_width_mm = 313u;
    g_snapshot.plan.profile.centre_offset_mm = 16;
    g_snapshot.plan.profile.b_shape_estimated = true;
    g_snapshot.plan.estimated_min_clearance_mm = 50u;

    bool const b_plan = scan_telemetry_get(&view)
                        && (1234u == view.captured_ms)
                        && (7u == view.request_id)
                        && (400u == view.forward_mm)
                        && (70u == view.forward_age_ms)
                        && (380u == view.nearest_observed_mm)
                        && (313u == view.observed_width_mm)
                        && (16 == view.centre_offset_mm)
                        && (50u == view.estimated_min_clearance_mm)
                        && (SCAN_TELEMETRY_SIDE_LEFT == view.bypass_side)
                        && (0u != (view.flags
                                   & SCAN_TELEMETRY_FORWARD_VALID))
                        && (0u != (view.flags
                                   & SCAN_TELEMETRY_PROFILE_SEEN))
                        && (0u != (view.flags
                                   & SCAN_TELEMETRY_SHAPE_ESTIMATE))
                        && (0u != (view.flags
                                   & SCAN_TELEMETRY_CLEARANCE_EST));

    g_snapshot.verdict = SCAN_TELEMETRY_STALE;
    g_snapshot.result = SCAN_TASK_RESULT_NONE;

    bool const b_stale = scan_telemetry_get(&view)
                         && (0u == view.request_id)
                         && (0u == view.forward_mm)
                         && (0u == view.obstacle_forward_mm)
                         && (0u == view.estimated_min_clearance_mm)
                         && (0u == view.flags);

    g_snapshot.flags = SCAN_TASK_FLAG_MONITORING
                       | SCAN_TASK_FLAG_REQUEST_PENDING;

    bool const b_pending = scan_telemetry_get(&view)
                           && (7u == view.request_id)
                           && (0u != (view.flags
                                      & SCAN_TELEMETRY_REQUEST_PENDING));

    g_snapshot.flags = SCAN_TASK_FLAG_MONITORING;

    g_snapshot.result = SCAN_TASK_RESULT_NO_ROUTE;
    g_snapshot.plan.profile.b_shape_estimated = false;

    bool const b_unknown = scan_telemetry_get(&view)
                           && (0u != (view.flags
                                      & SCAN_TELEMETRY_PROFILE_SEEN))
                           && (0u == (view.flags
                                      & SCAN_TELEMETRY_SHAPE_ESTIMATE))
                           && (0u == view.observed_width_mm)
                           && (SCAN_TELEMETRY_SIDE_NONE == view.bypass_side)
                           && (0u == view.estimated_min_clearance_mm);

    return ((b_busy && b_unready && b_plan && b_stale && b_pending && b_unknown)
            ? 0 : 1);
}

/* End of file. */
