/**
 * @file range_measurement.c
 * @brief HC-SR04 echo-duration conversion.
 */
#include "../include/range_measurement.h"

#include <stddef.h>

#define RANGE_ECHO_MIN_US          118u
#define RANGE_ECHO_MAX_US        23529u
#define RANGE_SOUND_FACTOR          17u
#define RANGE_SCALE_DIVISOR        100u

bool
range_measurement_from_echo (uint32_t echo_us,
                             uint16_t * p_distance_mm)
{
    bool b_is_valid = false;

    if ((NULL != p_distance_mm)
        && (RANGE_ECHO_MIN_US <= echo_us)
        && (RANGE_ECHO_MAX_US >= echo_us))
    {
        uint32_t const converted_mm =
            (echo_us * RANGE_SOUND_FACTOR) / RANGE_SCALE_DIVISOR;

        *p_distance_mm = (uint16_t)converted_mm;
        b_is_valid = true;
    }

    return (b_is_valid);
}

/* End of file. */
