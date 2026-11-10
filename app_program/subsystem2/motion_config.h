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

#ifndef S3_LINE_FOLLOW_TEST
#define S3_LINE_FOLLOW_TEST 0
#endif

/* Control period. The kernel tick is 10 ms (config.h CNF_TIMER_PERIOD),
   so 10 or 20 ms are the only exact choices. 10 ms: one odometry update
   per subsystem 4 IMU sample, and half the worst-case STOP latency. */
#define MC_PERIOD_MS            10u

/* Geometry. Counts per revolution are the provisional median of three
   S2-H03 hand-turned trials per wheel on 2026-11-04. Odometry uses only
   the ratio circumference / counts, so the circumference is the effective
   value from S2-H07 (2026-11-10): 204000 um x 1115 mm measured / 1001 mm
   odometry, three identical floor runs. The track is the effective value
   from S2-H08 (2026-11-10): 130 mm x 362.7 degrees odometry (median of
   three) / about 380 degrees measured for a commanded left 360 gave 124.
   124 and 122 still turned too far by eye; 120 ended 1-2 degrees past a
   full turn to the left, so 119 (1 mm is about 2.9 degrees per full
   turn, the finest step). */
#define MC_COUNTS_PER_REV       1330u       /* x2 counts per wheel rev */
#define MC_WHEEL_CIRC_UM        227233u     /* um, effective (S2-H07) */
#define MC_TRACK_MM             119u        /* effective (S2-H08) */

/* Wiring viewed from behind, ultrasonic sensor forward: left wheel = motor 1
   + encoder 2 (GP7/GP28), right wheel = motor 2 + encoder 1 (GP0/GP1).
   Positive duty drives motor A. Confirmed by a full line-follow lap on
   2026-11-10 (AGENTS.md, Agreed pin mapping). */
#define MC_LEFT_MOTOR           DRV_MOTOR_1
#define MC_RIGHT_MOTOR          DRV_MOTOR_2
#define MC_LEFT_MOTOR_INVERT    0           /* forward needs +duty (GP8) */
#define MC_RIGHT_MOTOR_INVERT   1           /* forward needs -duty (GP11) */
#define MC_LEFT_ENCODER         DRV_ENCODER_2
#define MC_RIGHT_ENCODER        DRV_ENCODER_1
#define MC_LEFT_ENC_INVERT      0           /* forward raw count increases */
#define MC_RIGHT_ENC_INVERT     1           /* forward raw count decreases */

/* Speed PID per wheel: mm/s in, duty out. UNMEASURED gains, tune on the
   robot. Gains are x PID_GAIN_SCALE (1024 = 1.0), per 10 ms sample. */
#define MC_PID_KP               8192        /* 8.0 duty per mm/s */
#define MC_PID_KI               1024        /* 1.0 */
#define MC_PID_KD               0
#define MC_SPEED_MAX_MM_S       400         /* UNMEASURED: below no-load max */
#define MC_SPEED_WINDOW         4u          /* speed over the last 4 periods */

/* The supervised line-follow profile uses an uncalibrated speed loop. Limit
   its authority so missing encoder feedback cannot drive the motors to full
   power. Normal firmware retains the full driver range. UNMEASURED: validate
   the minimum starting duty and tune the PID before increasing this limit. */
#if S3_LINE_FOLLOW_TEST
#define MC_CONTROL_DUTY_MAX     ((DRV_MOTOR_DUTY_MAX * 20) / 100)
#else
#define MC_CONTROL_DUTY_MAX     DRV_MOTOR_DUTY_MAX
#endif

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
#define MC_STALL_DUTY           ((MC_CONTROL_DUTY_MAX * 9) / 10)
#define MC_STALL_MS             500u        /* longer than spin-up */
#define MC_TIMEOUT_FACTOR       2u          /* x expected time ... */
#define MC_TIMEOUT_EXTRA_MS     1000u       /* ... + this */
#define MC_ENC_ERRORS_MAX       2u          /* decode errors per period */
#define MC_ENC_INVALID_MAX      3u          /* consecutive bad snapshots */

/* Targets. UNMEASURED tolerances; accuracy targets in AAI08_PostTri1
   are +/-2 cm per 50 cm and +/-5 degrees per 90 degree turn. */
#define MC_MOVE_TOL_MM          5u
#define MC_TURN_TOL_MDEG        2000u

/* Duty ramp: largest increase in |duty| per 10 ms period. UNMEASURED.
   Full scale in 200 ms (20 periods): both motors can no longer jump from
   0 to 100% in one step, which spreads the start-up current that most
   likely caused the S2-H04 freeze, yet it is short next to a manoeuvre
   (500 mm at 200 mm/s takes 2.5 s) and next to MC_STALL_MS. Decreases,
   STOP and Fault are never ramped. */
#define MC_DUTY_STEP_MAX        (DRV_MOTOR_DUTY_MAX / 20)

/* Hardware watchdog timeout. UNMEASURED: 20 control periods. Kicked only
   by Motion Control; not armed in TEST=1 or bench builds. */
#define MC_WATCHDOG_MS          200u

/* motion_q: 8 commands of 4 + 16 bytes (kernel message overhead). */
#define MC_MOTION_Q_BYTES       160

#endif /* MOTION_CONFIG_H */

/* End of file. */
