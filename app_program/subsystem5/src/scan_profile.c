/**
 * @file scan_profile.c
 * @brief Extract observed obstacle edges and span from a refined scan.
 * Subsystem 5 -- Timothy.  Student ID: 2501530.
 */
#include "../include/scan_profile.h"

#include <stddef.h>

/* sin(5..45 degrees) in thousandths. Range <=4000 mm keeps products in u32. */
static uint16_t const g_profile_sin_permille[SCAN_PROFILE_SIDE_BINS] =
{
    87u, 174u, 259u, 342u, 423u, 500u, 574u, 643u, 707u
};

/** Evaluate a side with both 5-degree readings around its first edge. */
static bool scan_profile_side (uint16_t const * p_distances,
                                   uint32_t pass_mm, uint32_t beam_steps,
                                   bool b_left, uint16_t * p_extent_mm);

/** The distance of the ray step steps from forward on one side. */
static uint16_t scan_profile_ray (uint16_t const * p_distances,
                                  bool b_left, uint32_t step);

/** Return a point's lateral distance from the forward sensor axis. */
static uint16_t scan_profile_lateral (uint32_t range_mm,
                                          uint32_t step);

scan_profile_result_t
scan_profile_make (uint16_t const * p_distances,
                       scan_profile_config_t const * p_config,
                       scan_profile_t * p_profile)
{
    bool b_valid = (NULL != p_distances) && (NULL != p_config)
                   && (NULL != p_profile);
    scan_profile_result_t result = SCAN_PROFILE_INVALID;

    if (NULL != p_profile)
    {
        scan_profile_t const cleared = {0};

        *p_profile = cleared;
    }

    if (b_valid)
    {
        b_valid = (0u < p_config->range_max_mm)
                  && (p_config->range_max_mm
                      <= SCAN_PROFILE_MAX_RANGE_MM)
                  && ((p_config->beam_deg % 5u) == 0u)
                  && (p_config->beam_deg <= SCAN_PROFILE_BEAM_MAX_DEG);

        for (uint32_t index = 0u;
             b_valid && (index < SCAN_PROFILE_ANGLES); index++)
        {
            uint16_t const value = p_distances[index];

            b_valid = (value <= p_config->range_max_mm)
                      && (((index % SCAN_PROFILE_COARSE_BINS) != 0u)
                          || (value > 0u));
        }
    }

    if (b_valid)
    {
        uint16_t const forward_mm =
            p_distances[SCAN_PROFILE_ANGLES / 2u];

        p_profile->forward_mm = forward_mm;

        if (forward_mm >= p_config->range_max_mm)
        {
            result = SCAN_PROFILE_CLEAR;
        }
        else
        {
            uint32_t const pass_mm = (uint32_t)forward_mm
                                     + (uint32_t)p_config->depth_mm;
            uint16_t nearest_mm = forward_mm;

            result = SCAN_PROFILE_OBSERVED;

            for (uint32_t index = 0u; index < SCAN_PROFILE_ANGLES;
                 index++)
            {
                uint16_t const value = p_distances[index];

                if ((0u < value) && ((uint32_t)value <= pass_mm)
                    && (value < nearest_mm))
                {
                    nearest_mm = value;
                }
            }

            p_profile->nearest_observed_mm = nearest_mm;
            uint32_t const beam_steps = (uint32_t)p_config->beam_deg / 5u;

            p_profile->b_left_edge = scan_profile_side(
                p_distances, pass_mm, beam_steps, true,
                &p_profile->left_extent_mm);
            p_profile->b_right_edge = scan_profile_side(
                p_distances, pass_mm, beam_steps, false,
                &p_profile->right_extent_mm);

            if (p_profile->b_left_edge && p_profile->b_right_edge
                && (0u < p_profile->left_extent_mm)
                && (0u < p_profile->right_extent_mm))
            {
                int32_t const left = (int32_t)p_profile->left_extent_mm;
                int32_t const right = (int32_t)p_profile->right_extent_mm;

                /* Each extent is <=4000 mm, so width fits uint16_t and
                   the signed centre offset fits int16_t. */
                p_profile->observed_width_mm = (uint16_t)(left + right);
                p_profile->centre_offset_mm = (int16_t)((left - right) / 2);
                p_profile->b_shape_estimated = true;
            }
        }
    }

    return (result);
}

/*
 * Ray steps are 5-degree units from forward. The first coarse ray past
 * the obstacle and the two fine rays before it locate the first ray that
 * misses (passes). A hit after a miss, or another obstacle on a later
 * coarse ray, makes the side unusable.
 *
 * Beam allowance (S2 decision 18): a ray hits anything within its beam,
 * so the miss means nothing reaches within the beam half-angle of it.
 * Each hit is therefore placed at most beam_steps inside the first miss.
 */
static bool
scan_profile_side (uint16_t const * p_distances, uint32_t pass_mm,
                       uint32_t beam_steps, bool b_left,
                       uint16_t * p_extent_mm)
{
    uint32_t coarse_pass = 0u;
    uint32_t miss_step = 0u;
    uint16_t extent_mm = 0u;
    bool b_valid = true;

    for (uint32_t step = SCAN_PROFILE_COARSE_BINS;
         (step <= SCAN_PROFILE_SIDE_BINS) && (0u == coarse_pass);
         step += SCAN_PROFILE_COARSE_BINS)
    {
        if ((uint32_t)scan_profile_ray(p_distances, b_left, step) > pass_mm)
        {
            coarse_pass = step;
        }
    }

    b_valid = (0u < coarse_pass);
    miss_step = coarse_pass;

    for (uint32_t step = coarse_pass - (SCAN_PROFILE_COARSE_BINS - 1u);
         b_valid && (step < coarse_pass); step++)
    {
        uint32_t const fine_mm = scan_profile_ray(p_distances, b_left, step);

        if (0u == fine_mm)
        {
            b_valid = false;
        }
        else if (fine_mm > pass_mm)
        {
            miss_step = (miss_step < coarse_pass) ? miss_step : step;
        }
        else if (miss_step < coarse_pass)
        {
            b_valid = false;  /* non-contiguous obstacle edge */
        }
        else
        {
            /* Still on the obstacle. */
        }
    }

    for (uint32_t step = coarse_pass + SCAN_PROFILE_COARSE_BINS;
         b_valid && (step <= SCAN_PROFILE_SIDE_BINS);
         step += SCAN_PROFILE_COARSE_BINS)
    {
        if ((uint32_t)scan_profile_ray(p_distances, b_left, step)
            <= pass_mm)
        {
            b_valid = false;  /* another obstacle beyond the first gap */
        }
    }

    if (b_valid)
    {
        uint32_t const cap = (miss_step > beam_steps)
                             ? (miss_step - beam_steps) : 0u;

        for (uint32_t step = 1u; step < miss_step; step++)
        {
            uint32_t const range_mm =
                scan_profile_ray(p_distances, b_left, step);

            if ((0u < range_mm) && (range_mm <= pass_mm))
            {
                uint16_t const lateral_mm = scan_profile_lateral(
                    range_mm, (step < cap) ? step : cap);

                extent_mm = (lateral_mm > extent_mm)
                            ? lateral_mm : extent_mm;
            }
        }
    }

    *p_extent_mm = b_valid ? extent_mm : 0u;

    return (b_valid);
}

bool
scan_profile_clear (uint16_t const * p_distances, uint16_t clear_mm)
{
    uint32_t const forward = SCAN_PROFILE_ANGLES / 2u;
    bool b_clear = (NULL != p_distances) && (0u < p_distances[forward]);

    for (uint32_t index = forward - SCAN_PROFILE_CLEAR_BINS;
         b_clear && (index <= (forward + SCAN_PROFILE_CLEAR_BINS)); index++)
    {
        b_clear = (0u == p_distances[index])
                  || (clear_mm <= p_distances[index]);
    }

    return (b_clear);
}

static uint16_t
scan_profile_ray (uint16_t const * p_distances, bool b_left, uint32_t step)
{
    return (b_left ? p_distances[(SCAN_PROFILE_ANGLES / 2u) + step]
                   : p_distances[(SCAN_PROFILE_ANGLES / 2u) - step]);
}

static uint16_t
scan_profile_lateral (uint32_t range_mm, uint32_t step)
{
    return ((0u == step) ? 0u
            : (uint16_t)((range_mm * g_profile_sin_permille[step - 1u])
                         / 1000u));
}

/* End of file. */
