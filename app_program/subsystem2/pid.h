/**
 * @file pid.h
 * @brief Subsystem 2 integer PID controller for wheel speed and distance.
 *
 * Pure logic: no hardware, kernel or clock access, so it is tested on the
 * Pico W with TEST=1 before any motor is connected. The caller runs
 * pid_update() once per fixed control period; ki and kd are per-sample
 * gains, so the period is folded into them rather than passed in.
 *
 * Gains are fixed point with PID_GAIN_SCALE as 1.0 (e.g. 1.5 = 1536).
 * Values are synthetic until tuned on the robot (TEST_LOG.md).
 */
#ifndef PID_H
#define PID_H

#include <stdbool.h>
#include <stdint.h>

#define PID_GAIN_SCALE  1024    /* fixed-point 1.0 for kp, ki and kd */
#define PID_INPUT_MAX   1000000 /* |setpoint| and |measurement| limit */
#define PID_GAIN_MAX    1048576 /* |gain| limit (1024.0), bounds products */

/** Tuning and output limits; checked by pid_init(). */
typedef struct
{
    int32_t kp;         /* proportional gain, PID_GAIN_SCALE = 1.0 */
    int32_t ki;         /* integral gain per sample */
    int32_t kd;         /* derivative gain per sample */
    int32_t out_min;    /* lowest output, e.g. -PWM top */
    int32_t out_max;    /* highest output, e.g. +PWM top */
} pid_cfg_t;

/** Controller state; treat as private and use only through pid_*(). */
typedef struct
{
    pid_cfg_t cfg;
    int64_t   i_sum;        /* integral term, scaled by PID_GAIN_SCALE */
    int32_t   prev_meas;    /* measurement from the previous update */
    bool      b_has_prev;   /* false until the first update */
} pid_ctrl_t;

bool pid_init (pid_ctrl_t * p_pid, pid_cfg_t const * p_cfg);
void pid_reset (pid_ctrl_t * p_pid);
bool pid_update (pid_ctrl_t * p_pid, int32_t setpoint, int32_t measurement,
                 int32_t * p_output);

#endif /* PID_H */

/* End of file. */
