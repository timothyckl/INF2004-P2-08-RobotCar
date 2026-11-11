/**
 * @file odometry.h
 * @brief Subsystem 2 wheel odometry: encoder counts to distance, speed and
 *        turn angle.
 *
 * Pure logic: no hardware, kernel or clock access, so it is tested on the
 * Pico W with TEST=1 before the encoders are wired. The encoder driver
 * keeps a free-running uint32_t count per wheel (incremented forward,
 * decremented in reverse, wrapping); odom_count_delta() turns two readings
 * into a signed change.
 *
 * Integer results truncate toward zero. Accumulate counts and convert the
 * total, rather than summing converted millimetres, so truncation does not
 * build up over a long run.
 *
 * Wheel and encoder parameters are not yet measured on the robot; tests use
 * synthetic values (TEST_LOG.md).
 */
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdbool.h>
#include <stdint.h>

#define ODOM_CPR_MAX      100000u   /* counts per wheel revolution limit */
#define ODOM_CIRC_UM_MAX  1000000u  /* wheel circumference limit, 1 m */
#define ODOM_TRACK_MM_MAX 1000u     /* wheel track limit */

/** Robot geometry; checked by odom_cfg_valid(). */
typedef struct
{
    uint32_t counts_per_rev;    /* encoder counts per wheel revolution */
    uint32_t wheel_circ_um;     /* wheel circumference, micrometres */
    uint32_t track_mm;          /* distance between wheel contact centres */
} odom_cfg_t;

bool    odom_cfg_valid (odom_cfg_t const * p_cfg);
int32_t odom_count_delta (uint32_t now, uint32_t prev);
bool    odom_counts_to_mm (odom_cfg_t const * p_cfg, int32_t counts,
                           int32_t * p_mm);
bool    odom_speed_mm_s (odom_cfg_t const * p_cfg, int32_t counts,
                         uint32_t interval_us, int32_t * p_speed);
bool    odom_turn_mdeg (odom_cfg_t const * p_cfg, int32_t left_mm,
                        int32_t right_mm, int32_t * p_mdeg);

#endif /* ODOMETRY_H */

/* End of file. */
