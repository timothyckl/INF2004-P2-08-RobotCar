/**
 * @file scan_profile.c
 * @brief Extract observed obstacle edges and span from a refined scan.
 * Subsystem 5 -- Timothy.  Student ID: 2501530.
 */
#include "../include/scan_profile.h"

#include <stddef.h>

/* sin(5..60 degrees) in thousandths. Range <=4000 mm keeps products in u32. */
static uint16_t const g_profile_sin_permille[12] =
{
    87u, 174u, 259u, 342u, 423u, 500u,
    574u, 643u, 707u, 766u, 819u, 866u
};

/** Evaluate a side with both 5-degree readings around its first edge. */
static bool scan_profile_side (uint16_t const * p_distances,
                                   uint32_t pass_mm, bool b_left,
                                   uint16_t * p_extent_mm);

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
                      <= SCAN_PROFILE_MAX_RANGE_MM);

        for (uint32_t index = 0u;
             b_valid && (index < SCAN_PROFILE_ANGLES); index++)
        {
            uint16_t const value = p_distances[index];

            b_valid = (value <= p_config->range_max_mm)
                      && (((index % 3u) != 0u) || (value > 0u));
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
            p_profile->b_left_edge = scan_profile_side(
                p_distances, pass_mm, true, &p_profile->left_extent_mm);
            p_profile->b_right_edge = scan_profile_side(
                p_distances, pass_mm, false, &p_profile->right_extent_mm);

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

static bool
scan_profile_side (uint16_t const * p_distances,
                       uint32_t pass_mm, bool b_left,
                       uint16_t * p_extent_mm)
{
    uint16_t extent_mm = 0u;
    uint32_t first_pass_step = 0u;
    bool b_passed = false;
    bool b_gap_seen = false;
    bool b_valid = true;

    for (uint32_t coarse_step = 1u;
         (coarse_step <= 4u) && (!b_passed); coarse_step++)
    {
        uint32_t const ray_step = coarse_step * 3u;
        uint32_t const index = b_left
                               ? ((SCAN_PROFILE_ANGLES / 2u) + ray_step)
                               : ((SCAN_PROFILE_ANGLES / 2u) - ray_step);
        uint32_t const range_mm = p_distances[index];

        if (range_mm > pass_mm)
        {
            b_passed = true;
            first_pass_step = coarse_step;

            for (uint32_t fine_step = ray_step - 2u;
                 fine_step < ray_step; fine_step++)
            {
                uint32_t const fine_index = b_left
                    ? ((SCAN_PROFILE_ANGLES / 2u) + fine_step)
                    : ((SCAN_PROFILE_ANGLES / 2u) - fine_step);
                uint32_t const fine_mm = p_distances[fine_index];

                if (0u == fine_mm)
                {
                    b_valid = false;
                }
                else if (fine_mm > pass_mm)
                {
                    b_gap_seen = true;
                }
                else if (b_gap_seen)
                {
                    b_valid = false;  /* non-contiguous obstacle edge */
                }
                else
                {
                    uint16_t const lateral_mm =
                        scan_profile_lateral(fine_mm, fine_step);

                    extent_mm = (lateral_mm > extent_mm)
                                ? lateral_mm : extent_mm;
                }
            }
        }
        else
        {
            /* Still on the obstacle: its lateral extent reaches here. */
            uint16_t const lateral_mm =
                scan_profile_lateral(range_mm, ray_step);

            extent_mm = (lateral_mm > extent_mm) ? lateral_mm : extent_mm;
        }
    }

    for (uint32_t coarse_step = first_pass_step + 1u;
         b_passed && (coarse_step <= 4u); coarse_step++)
    {
        uint32_t const ray_step = coarse_step * 3u;
        uint32_t const index = b_left
                               ? ((SCAN_PROFILE_ANGLES / 2u) + ray_step)
                               : ((SCAN_PROFILE_ANGLES / 2u) - ray_step);

        if (p_distances[index] <= pass_mm)
        {
            b_valid = false;  /* another obstacle beyond the first gap */
        }
    }

    *p_extent_mm = (b_valid && b_passed) ? extent_mm : 0u;

    return (b_valid && b_passed);
}

static uint16_t
scan_profile_lateral (uint32_t range_mm, uint32_t step)
{
    return ((uint16_t)((range_mm * g_profile_sin_permille[step - 1u])
                       / 1000u));
}

/* End of file. */
