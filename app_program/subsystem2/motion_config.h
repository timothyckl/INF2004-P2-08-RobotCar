/**
 * @file motion_config.h
 * @brief Subsystem 2 Motion Control tuning: the ONE place for its
 *        provisional numbers.
 *
 * Every value marked UNMEASURED is a starting guess so the code runs,
 * not a measurement. Replace each from the named test or the calibration
 * table in testing/subsystem2/TEST_LOG.md before relying on distances,
 * angles, speeds or fault timing. AGREED values come from
 * docs/decisions.md (subsystem 5 section 6).
 */
#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

#include "drv_encoder.h"
#include "drv_motor.h"

/* Control period. The kernel tick is 10 ms (config.h CNF_TIMER_PERIOD),
   so 10 or 20 ms are the only exact choices. 10 ms: one odometry update
   per subsystem 4 IMU sample, and half the worst-case STOP latency. */
#define MC_PERIOD_MS            10u

/* Geometry. UNMEASURED: counts per rev from S2-H03; wheel circumference
   and track from a ruler on the assembled robot. */
#define MC_COUNTS_PER_REV       40u         /* x2 counts per wheel rev */
#define MC_WHEEL_CIRC_UM        204000u     /* um */
#define MC_TRACK_MM             130u        /* wheel centre to centre */

/* Wiring. UNMEASURED: from S2-H01 (motor) and S2-H03 (encoder). */
#define MC_LEFT_MOTOR           DRV_MOTOR_1
#define MC_RIGHT_MOTOR          DRV_MOTOR_2
#define MC_LEFT_MOTOR_INVERT    0           /* 1: forward needs -duty */
#define MC_RIGHT_MOTOR_INVERT   0
#define MC_LEFT_ENCODER         DRV_ENCODER_1
#define MC_RIGHT_ENCODER        DRV_ENCODER_2
#define MC_LEFT_ENC_INVERT      0           /* 1: forward counts down */
#define MC_RIGHT_ENC_INVERT     0

/* Speed PID per wheel: mm/s in, duty out. UNMEASURED gains, tune on the
   robot. Gains are x PID_GAIN_SCALE (1024 = 1.0), per 10 ms sample. */
#define MC_PID_KP               8192        /* 8.0 duty per mm/s */
#define MC_PID_KI               1024        /* 1.0 */
#define MC_PID_KD               0
#define MC_SPEED_MAX_MM_S       400         /* UNMEASURED: below no-load max */
#define MC_SPEED_WINDOW         4u          /* speed over the last 4 periods */

/* Straight-line PID on the left/right distance difference: mm in, mm/s
   out. UNMEASURED gains, x PID_GAIN_SCALE, per 10 ms sample. */
#define MC_STRAIGHT_KP          2048        /* 2.0 (mm/s) per mm */
#define MC_STRAIGHT_KI          102         /* 0.1 */
#define MC_STRAIGHT_KD          0

/* Speeds the motion API uses. UNMEASURED: pick from PID tuning. */
#define MC_MOVE_SPEED_MM_S      200
#define MC_TURN_SPEED_MM_S      150

/* Stop confirmation. AGREED: 100 ms still, give up after 2 s.
   UNMEASURED: the "still" speed threshold, to calibrate. */
#define MC_STILL_MM_S           10u
#define MC_STILL_MS             100u
#define MC_STOP_LIMIT_MS        2000u

/* Faults. UNMEASURED. */
#define MC_STALL_DUTY           ((DRV_MOTOR_DUTY_MAX * 9) / 10)
#define MC_STALL_MS             500u        /* longer than spin-up */
#define MC_TIMEOUT_FACTOR       2u          /* x expected time ... */
#define MC_TIMEOUT_EXTRA_MS     1000u       /* ... + this */
#define MC_ENC_ERRORS_MAX       2u          /* decode errors per period */
#define MC_ENC_INVALID_MAX      3u          /* consecutive bad snapshots */

/* Targets. UNMEASURED tolerances; accuracy targets in AAI08_PostTri1
   are +/-2 cm per 50 cm and +/-5 degrees per 90 degree turn. */
#define MC_MOVE_TOL_MM          5u
#define MC_TURN_TOL_MDEG        2000u

/* motion_q: 8 commands of 4 + 16 bytes (kernel message overhead). */
#define MC_MOTION_Q_BYTES       160

#endif /* MOTION_CONFIG_H */

/* End of file. */
