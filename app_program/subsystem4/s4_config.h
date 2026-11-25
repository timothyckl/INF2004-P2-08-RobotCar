/**
 * @file s4_config.h
 * @brief Subsystem 4 terrain settings: the one place they are defined.
 *
 * The IMU task, the unit tests and the host replay tool
 * (testing/subsystem4/replay_raw.c) all take the mission settings from
 * s4_config_mission(), so a test can never check a copy of a value that
 * has since changed (review F-15). Pure: no kernel or hardware calls.
 */
#ifndef S4_CONFIG_H
#define S4_CONFIG_H

#include "terrain_logic.h"
#include "s4_motion_class.h"

/* About 1.5 s at the measured 29 ms per sample; the mean of 50 samples
 * with about 5 mg of noise is within about 1 mg. */
#define S4_CALIBRATION_SAMPLES      50u
#define S4_CALIBRATION_SPAN_MG      80u
#define S4_IMPACT_THRESHOLD_MG      1200u
#define S4_MAX_SAMPLE_GAP_MS        50u
#define S4_CONFIRMATION_SAMPLES     3u
#define S4_INVALID_SAMPLE_LIMIT     3u
/* A hump not ended within this is dropped (decision 10, provisional): at
 * line-follow speed (about 40 mm/s) a 195 mm hump takes about 5 s. */
#define S4_MAX_HUMP_MS              12000u

/* Tilt moving average (decision 10): 16 samples, about 470 ms at the
 * measured 29.5 ms per sample while line following. */
#define S4_AVERAGE_SAMPLES          16u

/* Contact geometry (decision 8). Track: S2 odometry track width (S2
 * decision, 119 mm). Wheelbase: wheel axle to the front ball caster,
 * measured with a ruler (88 mm). */
#define S4_TRACK_MM                 119u
#define S4_WHEELBASE_MM             88u

/* Hump hysteresis on the uncalibrated height (decision 17): a hump starts
 * at 6 mm (decision 11; floor undulations read up to about 4 mm while
 * driving, a 10 mm hump 11 to 12 mm) and ends below 2 mm. Replayed
 * through terrain_logic (testing/subsystem4/replay_raw.c), the six logged
 * drives with detection on peak at 4.0 mm while following off the hump
 * and 10.8 to 12.5 mm over the 10 mm hump (six crossings). Tilt noise at
 * rest is about 10 mg, about 1 mm of height. Until 2026-11-25 the
 * thresholds compared the scaled height, so the 0.851 scale of decision
 * 12 had raised the real start to 7.05 mm without anyone choosing it. */
#define S4_RISE_UM                  6000u
#define S4_LEVEL_UM                 2000u

/* Reported-height correction (decision 12): the 10 mm hump read a mean of
 * 11.75 mm over four crossings (S4-H08), so 10 / 11.75 = 0.851. It and
 * the offset apply only to the reported peak, never to detection. The
 * offset of a two-point calibration (decision 17) is 0 until reference
 * humps of several heights have been crossed several times each; the
 * procedure and the fitting tool are in testing/subsystem4/README.md. */
#define S4_HEIGHT_SCALE_PERMILLE    851u
#define S4_HEIGHT_OFFSET_UM         0u

/* Lift detection (decision 15, provisional), with the hump gate above.
 * In logs/2026-11-16_s4-raw-d14.txt, about 2900 driving samples read at
 * most +106 mg up and 97 mg of averaged tilt; in seven handling episodes
 * a single sample reached +209 to +622 mg up (rarely two in a row), and
 * the averaged tilt 309 to 478 mg in most. 300 mg of tilt is about
 * 17 degrees, or a 36 mm rise under one wheel. The averaged tilt took
 * about 0.5 s to reach it (S4-H11), so an unaveraged tilt of 200 mg held
 * for 3 samples (about 70 ms) also counts: driving jolts, such as leaving
 * the hump, held it for at most 2 in about 3300 samples. */
#define S4_LIFT_UP_MG               200u
#define S4_LIFT_SAMPLES             1u
#define S4_LIFT_TILT_MG             300u
#define S4_LIFT_RAW_TILT_MG         200u
#define S4_LIFT_RAW_SAMPLES         3u

/* Hump, impact and lift detection (decisions 12, 14, 16, 17): only while
 * the robot follows the line (FOLLOW), and not for this many samples after
 * it starts to (about 0.7 s at 21 ms per sample), which covers the GP20
 * press, a junction turn or a bypass, and lets the 16-sample average clear
 * it. */
#define S4_SETTLE_SAMPLES           32u

/* Forward acceleration reads as pitch (200 mg, 17 mm of caster height,
 * when Motion Control ramps to speed; decision 8). Pitch is ignored while
 * the mean wheel speed changes by more than this between samples, and for
 * a hold time after (provisional). The same window is the motion
 * classifier's accelerating or braking. */
#define S4_PITCH_DV_MM_S            15
#define S4_PITCH_HOLD_SAMPLES       8u

/* Motion classification (decision 20, observational, provisional):
 * - stationary: both wheels at or below Motion Control's stillness limit
 *   (MC_STILL_MM_S, motion_config.h; task_imu.c checks they match);
 * - turning: encoder yaw rate of at least 8 deg/s. Line correction differs
 *   the wheels by at most 8 mm/s (40 against 32 mm/s, S3 decision 4),
 *   3.9 deg/s on the 119 mm track; a sustained curve (35 against 5 mm/s)
 *   is 14.4 deg/s, and the junction and bypass spins (30 mm/s each way,
 *   CTRL_BYPASS_SPIN_MM_S) about 29 deg/s;
 * - accelerating or braking: the pitch gate's window above. Starting to
 *   follow at 40 mm/s changes the mean speed by less than 15 mm/s per
 *   sample and reads as cruising; a STOP from line speed is still within
 *   a sample or two and reads as stationary;
 * - descending: on a hump, more than the detector's own hysteresis band
 *   (S4_RISE_UM - S4_LEVEL_UM) below the peak;
 * - unknown: the Motion Control snapshot is older than the terrain logic's
 *   sample gap (S4_MAX_SAMPLE_GAP_MS);
 * - a new state is taken after S4_CONFIRMATION_SAMPLES samples. */
#define S4_MOTION_STILL_MM_S        10u
#define S4_MOTION_TURN_DDEG_S       80u

/** The terrain settings the mission image runs with. */
s4_terrain_config_t s4_config_mission(void);

/** The motion-classification settings the mission image runs with. */
s4_motion_class_config_t s4_motion_config_mission(void);

#endif /* S4_CONFIG_H */

/* End of file. */
