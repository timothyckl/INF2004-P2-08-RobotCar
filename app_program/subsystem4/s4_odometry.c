/**
 * @file s4_odometry.c
 * @brief Encoder distance for terrain processing (see s4_odometry.h).
 */
#include "s4_odometry.h"

#include "../subsystem2/subsystem2.h"

void
s4_odometry_init (s4_odometry_t * p_odometry)
{
    if (NULL != p_odometry)
    {
        p_odometry->b_have_previous = false;
    }
}

void
s4_odometry_step (s4_odometry_t * p_odometry,
                  motion_status_t const * p_status,
                  uint32_t sample_ms,
                  s4_odometry_step_t * p_step)
{
    if ((NULL == p_odometry) || (NULL == p_step))
    {
        return;
    }

    p_step->delta_mm = 0u;
    p_step->b_valid = false;
    p_step->b_new_run = false;

    if (NULL == p_status)
    {
        p_odometry->b_have_previous = false;
        return;
    }

    /* Unsigned age: a status from after the sample also counts as aged,
       and the comparison stays correct across the 32-bit wrap. */
    uint32_t const age_ms = sample_ms - p_status->captured_ms;
    bool const b_aligned = (age_ms <= S4_ODOMETRY_MAX_AGE_MS);

    if (p_odometry->b_have_previous &&
        (p_odometry->previous.run_id != p_status->run_id))
    {
        p_step->b_new_run = true;
    }
    else if (p_odometry->b_have_previous && b_aligned)
    {
        p_step->b_valid = s2_distance_delta(&p_odometry->previous,
                                            p_status, &p_step->delta_mm);
    }
    else
    {
        /* First status, or not aligned with the sample: no distance. */
    }

    p_odometry->previous = *p_status;
    p_odometry->b_have_previous = true;
}

/* End of file. */
