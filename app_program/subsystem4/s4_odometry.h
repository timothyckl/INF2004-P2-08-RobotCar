/**
 * @file s4_odometry.h
 * @brief Encoder distance for terrain processing, from subsystem 2.
 *
 * Follows subsystem2.h: each IMU sample reads motion_get_status() and
 * takes the forward distance since the previous status with
 * s2_distance_delta(). A run_id change means the controller started a new
 * run, so the terrain run maximum is reset (design review: controller-
 * authorised new-run reset). A distance is used only when the status was
 * captured within S4_ODOMETRY_MAX_AGE_MS of the IMU sample, so tilt and
 * distance describe the same moment. Pure logic: no kernel calls.
 */
#ifndef S4_ODOMETRY_H
#define S4_ODOMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "../subsystem2/motion_api.h"

/* Two Motion Control periods (10 ms each). */
#define S4_ODOMETRY_MAX_AGE_MS  20u

typedef struct
{
    motion_status_t previous;
    bool            b_have_previous;
} s4_odometry_t;

typedef struct
{
    uint16_t delta_mm;      /* forward distance since the last sample */
    bool     b_valid;       /* pass delta_mm to s4_terrain_process() */
    bool     b_new_run;     /* call s4_terrain_start_new_run() first */
} s4_odometry_step_t;

void s4_odometry_init(s4_odometry_t * p_odometry);

/**
 * @brief Distance for one IMU sample.
 *
 * @param p_odometry State owned by the IMU task.
 * @param p_status Latest Motion Control status, or NULL if it could not
 *                 be read (the distance is then invalid).
 * @param sample_ms Time of the IMU sample (tk_get_tim, low 32 bits).
 * @param p_step Receives the result.
 */
void s4_odometry_step(s4_odometry_t * p_odometry,
                      motion_status_t const * p_status,
                      uint32_t sample_ms,
                      s4_odometry_step_t * p_step);

#endif /* S4_ODOMETRY_H */

/* End of file. */
