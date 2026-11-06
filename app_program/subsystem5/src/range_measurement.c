/**
 * @file range_measurement.c
 * @brief HC-SR04 echo-duration conversion.
 *
 * Source: docs/datasheets/HCSR04-datasheet.pdf and obstacle-scanning
 * decision 3 in docs/decisions.md.
 */
#include "../include/range_measurement.h"

#include <stddef.h>

static uint32_t const g_range_echo_min_us   = 118u;
static uint32_t const g_range_echo_max_us   = 23530u;
static uint32_t const g_range_sound_factor  = 17u;
static uint32_t const g_range_scale_divisor = 100u;

bool
range_measurement_from_echo (uint32_t echo_us,
                             uint16_t * p_distance_mm)
{
    bool b_is_valid = false;

    if ((NULL != p_distance_mm)
        && (g_range_echo_min_us <= echo_us)
        && (g_range_echo_max_us >= echo_us))
    {
        uint32_t const converted_mm =
            (echo_us * g_range_sound_factor) / g_range_scale_divisor;

        /* The validated echo range limits the result to 4000 mm. */
        *p_distance_mm = (uint16_t) converted_mm;
        b_is_valid     = true;
    }

    return (b_is_valid);
}

/* End of file. */
