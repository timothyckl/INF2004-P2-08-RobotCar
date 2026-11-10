/**
 * @file scan_telemetry.c
 * @brief Map one S5 task snapshot to a decoded, kernel-free view.
 * Subsystem 5 -- Timothy.  Student ID: 2501530.
 */
#include "../include/scan_telemetry.h"

#include "../include/forward_monitor.h"
#include "../include/scan_task.h"

/* Keep the public decoded codes aligned with S5's internal values. */
typedef char scan_telemetry_codes_match[
    ((SCAN_TELEMETRY_SAFE == FORWARD_MONITOR_SAFE)
     && (SCAN_TELEMETRY_UNSAFE == FORWARD_MONITOR_UNSAFE)
     && (SCAN_TELEMETRY_INVALID == FORWARD_MONITOR_INVALID)
     && (SCAN_TELEMETRY_STALE == FORWARD_MONITOR_STALE)
     && (SCAN_TELEMETRY_NONE == SCAN_TASK_RESULT_NONE)
     && (SCAN_TELEMETRY_PLAN == SCAN_TASK_RESULT_PLAN)
     && (SCAN_TELEMETRY_CLEAR == SCAN_TASK_RESULT_CLEAR)
     && (SCAN_TELEMETRY_NO_ROUTE == SCAN_TASK_RESULT_NO_ROUTE)
     && (SCAN_TELEMETRY_FAULT == SCAN_TASK_RESULT_FAULT)
     && (SCAN_TELEMETRY_CANCELLED == SCAN_TASK_RESULT_CANCELLED)
     && (SCAN_TELEMETRY_SIDE_LEFT == BYPASS_PLAN_LEFT)
     && (SCAN_TELEMETRY_SIDE_RIGHT == BYPASS_PLAN_RIGHT)) ? 1 : -1];

bool
scan_telemetry_get (scan_telemetry_t * p_out)
{
    bool b_success = false;

    if (NULL != p_out)
    {
        scan_task_status_t snapshot;

        if ((E_OK == scan_task_get(&snapshot))
            && (0u != (snapshot.flags & (SCAN_TASK_FLAG_MONITORING
                                         | SCAN_TASK_FLAG_SCANNING))))
        {
            scan_telemetry_t view = {0};
            bool const b_forward_valid =
                ((FORWARD_MONITOR_SAFE == snapshot.verdict)
                 || (FORWARD_MONITOR_UNSAFE == snapshot.verdict))
                && (0u < snapshot.forward_mm)
                && (snapshot.forward_mm <= SCAN_PROFILE_MAX_RANGE_MM);
            bool const b_profile_result =
                (SCAN_TASK_RESULT_PLAN == snapshot.result)
                || (SCAN_TASK_RESULT_NO_ROUTE == snapshot.result);

            view.captured_ms     = snapshot.captured_ms;
            view.forward_verdict = snapshot.verdict;
            view.scan_result     = snapshot.result;

            if ((0u != (snapshot.flags & SCAN_TASK_FLAG_REQUEST_PENDING))
                || (SCAN_TASK_RESULT_PLAN == snapshot.result)
                || (SCAN_TASK_RESULT_CLEAR == snapshot.result)
                || (SCAN_TASK_RESULT_NO_ROUTE == snapshot.result)
                || (SCAN_TASK_RESULT_CANCELLED == snapshot.result))
            {
                view.request_id = snapshot.request_id;
            }

            if (0u != (snapshot.flags & SCAN_TASK_FLAG_SCANNING))
            {
                view.flags |= SCAN_TELEMETRY_SCANNING;
            }

            if (0u != (snapshot.flags & SCAN_TASK_FLAG_REQUEST_PENDING))
            {
                view.flags |= SCAN_TELEMETRY_REQUEST_PENDING;
            }

            if (b_forward_valid)
            {
                view.forward_mm = snapshot.forward_mm;
                view.forward_age_ms = snapshot.forward_age_ms;
                view.flags |= SCAN_TELEMETRY_FORWARD_VALID;
            }

            if (b_profile_result
                && (0u < snapshot.plan.profile.forward_mm))
            {
                view.obstacle_forward_mm = snapshot.plan.profile.forward_mm;
                view.nearest_observed_mm =
                    snapshot.plan.profile.nearest_observed_mm;
                view.flags |= SCAN_TELEMETRY_PROFILE_SEEN;

                if (snapshot.plan.profile.b_shape_estimated)
                {
                    view.observed_width_mm =
                        snapshot.plan.profile.observed_width_mm;
                    view.centre_offset_mm =
                        snapshot.plan.profile.centre_offset_mm;
                    view.flags |= SCAN_TELEMETRY_SHAPE_ESTIMATE;
                }
            }

            if (SCAN_TASK_RESULT_PLAN == snapshot.result)
            {
                view.bypass_side = snapshot.plan.side;
                view.estimated_min_clearance_mm =
                    snapshot.plan.estimated_min_clearance_mm;
                view.flags |= SCAN_TELEMETRY_CLEARANCE_EST;
            }

            *p_out = view;
            b_success = true;
        }
    }

    return (b_success);
}

/* End of file. */
