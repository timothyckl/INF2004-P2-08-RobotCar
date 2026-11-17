/**
 * @file s2_s1_telemetry.c
 * @brief robot_read_telemetry() for S1's service_bridge, in a kernel-free
 *        file because S1's headers cannot share one with the kernel's
 *        (size_t conflict, see s2_s1_adapter.h).
 *
 * Compiled to nothing until subsystem 1's telemetry_encoder.h is in the
 * tree (after the hy_wifi_telemetry merge); then it matches
 * service_bridge_snapshot_fn_t with no change here.
 */
#if defined(__has_include)
#if __has_include("../subsystem1/telemetry_encoder.h")
#define S2_HAVE_S1_TELEMETRY 1
#endif
#endif

#ifdef S2_HAVE_S1_TELEMETRY

#include "../subsystem1/telemetry_encoder.h"
#include "s2_s1_adapter.h"

bool
robot_read_telemetry (void * p_context,
                      telemetry_encoder_snapshot_t * p_snapshot)
{
    s2_telemetry_t t;

    (void)p_context;

    if ((NULL == p_snapshot) || (!s2_read_telemetry(&t)))
    {
        return (false);
    }

    p_snapshot->captured_ms    = t.captured_ms;
    p_snapshot->distance_mm    = t.distance_mm;
    p_snapshot->distance_valid = t.b_distance_valid;
    p_snapshot->stopped        = t.b_stopped;
    p_snapshot->link_up        = false;    /* S1 fills its own link state */

    return (true);
}

#else

/* ISO C forbids an empty translation unit. */
typedef int s2_s1_telemetry_unused_t;

#endif /* S2_HAVE_S1_TELEMETRY */

/* End of file. */
