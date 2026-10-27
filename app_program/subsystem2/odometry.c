/**
 * @file odometry.c
 * @brief Subsystem 2 wheel odometry (see odometry.h).
 *
 * Every product is formed in int64_t. With the limits in odometry.h the
 * largest intermediate (speed numerator) is below 2^31 * 10^6 * 10^3,
 * about 2.1e18, inside int64_t. Results that do not fit int32_t are
 * rejected instead of wrapping.
 */
#include "odometry.h"

#include <stddef.h>

#define ODOM_UM_PER_MM    1000
#define ODOM_MS_PER_S     1000      /* um/us to mm/s scale */
#define ODOM_MDEG_HALF    180000    /* 180 degrees in millidegrees */
#define ODOM_PI_NUM       355       /* pi ~= 355 / 113, error < 1e-6 */
#define ODOM_PI_DEN       113

static bool odom_fits_i32 (int64_t value);

/**
 * @brief Check geometry is non-zero and within the documented limits.
 */
bool
odom_cfg_valid (odom_cfg_t const * p_cfg)
{
    return ((NULL != p_cfg)
            && (p_cfg->counts_per_rev > 0u)
            && (p_cfg->counts_per_rev <= ODOM_CPR_MAX)
            && (p_cfg->wheel_circ_um > 0u)
            && (p_cfg->wheel_circ_um <= ODOM_CIRC_UM_MAX)
            && (p_cfg->track_mm > 0u)
            && (p_cfg->track_mm <= ODOM_TRACK_MM_MAX));
}

/**
 * @brief Signed change between two free-running counter readings.
 *
 * Correct across counter wraparound provided the true change is within
 * INT32_MIN..INT32_MAX. Avoids converting an out-of-range unsigned value
 * to int32_t, which is implementation-defined (C99 6.3.1.3).
 */
int32_t
odom_count_delta (uint32_t now, uint32_t prev)
{
    uint32_t const diff = now - prev;
    int32_t        delta;

    if (diff <= (uint32_t)INT32_MAX)
    {
        delta = (int32_t)diff;
    }
    else
    {
        delta = -(int32_t)(UINT32_MAX - diff) - 1;
    }

    return (delta);
}

/**
 * @brief Convert a signed count to millimetres travelled by that wheel.
 * @return false, leaving *p_mm unchanged, on invalid input or overflow.
 */
bool
odom_counts_to_mm (odom_cfg_t const * p_cfg, int32_t counts, int32_t * p_mm)
{
    if ((NULL == p_mm) || (!odom_cfg_valid(p_cfg)))
    {
        return (false);
    }

    int64_t const mm = ((int64_t)counts * p_cfg->wheel_circ_um)
                       / ((int64_t)p_cfg->counts_per_rev * ODOM_UM_PER_MM);

    if (!odom_fits_i32(mm))
    {
        return (false);
    }

    *p_mm = (int32_t)mm;

    return (true);
}

/**
 * @brief Wheel speed from counts seen over an interval.
 * @return false, leaving *p_speed unchanged, on invalid input, a zero
 *         interval or overflow.
 */
bool
odom_speed_mm_s (odom_cfg_t const * p_cfg, int32_t counts,
                 uint32_t interval_us, int32_t * p_speed)
{
    if ((NULL == p_speed) || (0u == interval_us) || (!odom_cfg_valid(p_cfg)))
    {
        return (false);
    }

    /* mm/s = (counts * circ_um / cpr) um / interval_us us * 1000 */
    int64_t const speed = ((int64_t)counts * p_cfg->wheel_circ_um
                           * ODOM_MS_PER_S)
                          / ((int64_t)p_cfg->counts_per_rev * interval_us);

    if (!odom_fits_i32(speed))
    {
        return (false);
    }

    *p_speed = (int32_t)speed;

    return (true);
}

/**
 * @brief Heading change of a differential-drive vehicle, in millidegrees.
 *
 * Positive is a left (anticlockwise) turn, matching subsystem 5's angle
 * convention (docs/decisions.md, subsystem 5 section 4).
 * @return false, leaving *p_mdeg unchanged, on invalid input or overflow.
 */
bool
odom_turn_mdeg (odom_cfg_t const * p_cfg, int32_t left_mm, int32_t right_mm,
                int32_t * p_mdeg)
{
    if ((NULL == p_mdeg) || (!odom_cfg_valid(p_cfg)))
    {
        return (false);
    }

    /* angle_rad = (right - left) / track; degrees = rad * 180 / pi */
    int64_t const diff = (int64_t)right_mm - left_mm;
    int64_t const mdeg = (diff * ODOM_MDEG_HALF * ODOM_PI_DEN)
                         / ((int64_t)p_cfg->track_mm * ODOM_PI_NUM);

    if (!odom_fits_i32(mdeg))
    {
        return (false);
    }

    *p_mdeg = (int32_t)mdeg;

    return (true);
}

static bool
odom_fits_i32 (int64_t value)
{
    return ((value >= INT32_MIN) && (value <= INT32_MAX));
}

/* End of file. */
