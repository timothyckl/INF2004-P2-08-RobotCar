/**
 * @file forward_monitor.c
 * @brief Forward clearance verdict (see forward_monitor.h).
 */
#include "../include/forward_monitor.h"

#include <stddef.h>

void
forward_monitor_reset (forward_monitor_t * p_monitor)
{
    if (NULL != p_monitor)
    {
        forward_monitor_t const cleared = {0};

        *p_monitor = cleared;
    }

}

void
forward_monitor_sample (forward_monitor_t * p_monitor, bool b_valid,
                        uint16_t distance_mm, uint32_t now_us)
{
    if (NULL != p_monitor)
    {
        p_monitor->sample_us    = now_us;
        p_monitor->b_has_sample = true;
        p_monitor->b_is_valid   = b_valid;
        p_monitor->distance_mm  = b_valid ? distance_mm : 0u;
    }

}

forward_monitor_verdict_t
forward_monitor_check (forward_monitor_t const * p_monitor,
                       forward_monitor_config_t const * p_config,
                       uint32_t now_us)
{
    forward_monitor_verdict_t verdict = FORWARD_MONITOR_STALE;

    if ((NULL != p_monitor) && (NULL != p_config)
        && (p_monitor->b_has_sample)
        && ((now_us - p_monitor->sample_us) <= p_config->fresh_us))
    {
        if (!p_monitor->b_is_valid)
        {
            verdict = FORWARD_MONITOR_INVALID;
        }
        else if (p_monitor->distance_mm < p_config->hazard_mm)
        {
            verdict = FORWARD_MONITOR_UNSAFE;
        }
        else
        {
            verdict = FORWARD_MONITOR_SAFE;
        }

    }

    return (verdict);
}

/* End of file. */
