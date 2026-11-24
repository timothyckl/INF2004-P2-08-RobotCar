/**
 * @file task_imu.c
 * @brief Periodic IMU sampling and terrain processing task.
 */
#include "subsystem4_internal.h"
#include "../subsystem2/motion_api.h"   /* motion_get_status (IF-17) */
#include "../subsystem2/subsystem2.h"   /* s2_get_robot_state (IF-14) */

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

/* Hump hysteresis: a hump starts at 6 mm (decision 11; floor undulations
 * read up to about 4 mm while driving, a 10 mm hump 11 to 12 mm) and ends
 * below 2 mm. Tilt noise at rest is about 10 mg, about 1 mm of height. */
#define S4_RISE_UM                  6000u
#define S4_LEVEL_UM                 2000u

/* Forward acceleration reads as pitch (200 mg, 17 mm of caster height,
 * when Motion Control ramps to speed). Pitch is ignored while the mean
 * wheel speed changes by more than this between samples, and for a hold
 * time after (provisional). */
#define S4_PITCH_DV_MM_S            15
#define S4_PITCH_HOLD_SAMPLES       8u

/* Hump and impact detection (decisions 12, 14): only while the robot
 * drives (FOLLOW,
 * NAVIGATE, BYPASS or REACQUIRE), and not for this many samples after it
 * starts to (about 0.7 s at 21 ms per sample), which covers the GP20 press
 * and lets the 16-sample average clear it. */
#define S4_SETTLE_SAMPLES           32u

/* Measured-height correction (decision 12): the 10 mm hump read a mean of
 * 11.75 mm over four crossings (S4-H08), so 10 / 11.75 = 0.851. */
#define S4_HEIGHT_SCALE_PERMILLE    851u

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

#if S4_RAW_LOG
/* Diagnostic (S4_RAW_LOG only): the loop step the IMU task is in and a
 * count of completed loops, written only by the IMU task and read by
 * s4_watch_task. Single aligned writes, so the reader sees whole values. */
static volatile uint8_t  g_s4_diag_stage = 0u;
static volatile uint32_t g_s4_diag_loops = 0u;

#define S4_WATCH_MAX                3u

typedef struct
{
    ID                        task_id;
    volatile uint32_t const * p_loops;
    char const *              p_name;
} s4_watch_entry_t;

/* Entries are written before the count is raised, by one task. */
static s4_watch_entry_t g_s4_watch_list[S4_WATCH_MAX];
static volatile uint8_t g_s4_watch_count = 0u;
#define S4_DIAG_STAGE(stage)    (g_s4_diag_stage = (uint8_t) (stage))
#else
#define S4_DIAG_STAGE(stage)    ((void) 0)
#endif

static s4_terrain_config_t s4_runtime_config(void);
static uint32_t s4_capture_time_us(void);
static void s4_mark_sensor_error(s4_terrain_context_t * p_context,
                                 uint32_t capture_time_us);
static void s4_sample_to_vehicle(s4_terrain_sample_t * p_sample);

/* Motion Control snapshot from the previous sample (IF-17). */
typedef struct
{
    int32_t speed_mm_s;
    uint8_t run_id;
    uint8_t pitch_hold;
    bool    b_have_previous;
} s4_motion_t;

static bool s4_motion_read(s4_motion_t * p_motion,
                           s4_terrain_context_t * p_terrain);
static bool s4_detect_gate(uint8_t * p_settle);
#if S4_RAW_LOG
static void s4_raw_log(s4_terrain_sample_t const * p_vehicle,
                       s4_terrain_result_t const * p_result,
                       bool b_pitch_valid,
                       bool b_detect_enabled);
#endif
static void s4_report_events(s4_terrain_result_t const * p_result,
                             uint32_t calibration_start_us);

void
s4_imu_task (INT start_code,
             void * p_exinf)
{
    s4_lsm303_driver_t driver;
    s4_terrain_context_t terrain;
    s4_terrain_config_t config = s4_runtime_config();
    s4_terrain_sample_t sample = {0};
    uint32_t calibration_start_us = 0u;
    s4_motion_t motion = {0};
    bool b_pitch_valid = false;
    bool b_detect_enabled = false;
    uint8_t settle = S4_SETTLE_SAMPLES;
    ER error;

    (void) start_code;
    (void) p_exinf;

    s4_lsm303_init(&driver);
    s4_terrain_init(&terrain, &config);
    (void) s4_status_publish(&terrain.result);

    for (;;)
    {
        if (!driver.b_open)
        {
            s4_terrain_request_recovery(&terrain);
            S4_DIAG_STAGE(1u);
            error = s4_lsm303_open(&driver);
            if (error < E_OK)
            {
                tm_printf((UB *) "s4: IMU startup failed (%d), retrying\n",
                          error);
                s4_mark_sensor_error(&terrain, s4_capture_time_us());
                (void) s4_status_publish(&terrain.result);
                (void) tk_dly_tsk(S4_RETRY_PERIOD_MS);
                continue;
            }

            tm_printf((UB *) "s4: LSM303DLHC ready; calibrating\n");
            calibration_start_us = s4_capture_time_us();
        }

        S4_DIAG_STAGE(2u);
        error = s4_lsm303_read_acceleration(&driver,
                                            s4_capture_time_us(),
                                            &sample);
        if (E_OK == error)
        {
            s4_sample_to_vehicle(&sample);
            S4_DIAG_STAGE(3u);
            b_pitch_valid = s4_motion_read(&motion, &terrain);
            b_detect_enabled = s4_detect_gate(&settle);
            s4_terrain_set_detect_enable(&terrain, b_detect_enabled);
            S4_DIAG_STAGE(4u);
            s4_terrain_process(&terrain, &sample, b_pitch_valid);
#if S4_RAW_LOG
            S4_DIAG_STAGE(5u);
            s4_raw_log(&sample, &terrain.result, b_pitch_valid,
                       b_detect_enabled);
#endif
            S4_DIAG_STAGE(6u);
            s4_report_events(&terrain.result, calibration_start_us);
            S4_DIAG_STAGE(7u);
            (void) s4_status_publish(&terrain.result);
        }
        else if (E_BUSY != error)
        {
            /* E_IO and E_TMOUT are I2C errors (kernel I2C driver). */
            tm_printf((UB *) "s4: IMU read failed (%d)\n", error);
            s4_mark_sensor_error(&terrain, s4_capture_time_us());
            (void) s4_status_publish(&terrain.result);
        }
        else
        {
            /* No new sample yet; the next poll checks again. */
        }

        if ((S4_TERRAIN_STATE_FAULT == terrain.result.state) &&
            (S4_TERRAIN_FAULT_CALIBRATION == terrain.result.fault))
        {
            /* The car moved while calibrating: the sensor is fine, so
             * calibrate again without restarting it. */
            tm_printf((UB *) "s4: calibration failed, car moved (span "
                      "fwd %d left %d up %d mg); keep still\n",
                      (INT) terrain.calibration_max_x -
                      terrain.calibration_min_x,
                      (INT) terrain.calibration_max_y -
                      terrain.calibration_min_y,
                      (INT) terrain.calibration_max_z -
                      terrain.calibration_min_z);
            s4_terrain_request_recovery(&terrain);
            (void) s4_status_publish(&terrain.result);
            calibration_start_us = s4_capture_time_us();
            (void) tk_dly_tsk(S4_POLL_PERIOD_MS);
        }
        else if (S4_TERRAIN_STATE_FAULT == terrain.result.state)
        {
            tm_printf((UB *) "s4: terrain fault %u; sensor restart\n",
                      (UINT) terrain.result.fault);
            (void) s4_lsm303_close(&driver);
            (void) tk_dly_tsk(S4_RETRY_PERIOD_MS);
        }
        else
        {
            S4_DIAG_STAGE(8u);
            (void) tk_dly_tsk(S4_POLL_PERIOD_MS);
        }
#if S4_RAW_LOG
        g_s4_diag_loops++;
#endif
    }
}

#if S4_RAW_LOG
void
s4_watch_add (ID task_id,
              volatile uint32_t const * p_loops,
              char const * p_name)
{
    uint8_t const count = g_s4_watch_count;

    if ((count < S4_WATCH_MAX) && (NULL != p_loops) && (NULL != p_name))
    {
        g_s4_watch_list[count].task_id = task_id;
        g_s4_watch_list[count].p_loops = p_loops;
        g_s4_watch_list[count].p_name = p_name;
        /* count is below S4_WATCH_MAX (3) here, so count + 1 fits uint8_t. */
        g_s4_watch_count = (uint8_t) (count + 1u);
    }
}

void
s4_watch_add_imu (ID task_id)
{
    s4_watch_add(task_id, &g_s4_diag_loops, "s4 imu");
}

/*
 * Diagnostic task (S4_RAW_LOG only). Once a second, for each registered
 * task whose loop count has not advanced, prints its kernel wait state
 * (tk_ref_tsk: tskstat, tskwait factor TTW_*, wait object ID) and, for the
 * IMU, the loop stage: 1 open, 2 sensor read, 3 motion status, 4 terrain
 * logic, 5 raw log, 6 events, 7 status publish, 8 poll delay.
 */
void
s4_watch_task (INT start_code,
               void * p_exinf)
{
    uint32_t last[S4_WATCH_MAX] = {0u};

    (void) start_code;
    (void) p_exinf;

    for (;;)
    {
        uint8_t const count = g_s4_watch_count;

        for (uint8_t index = 0u; index < count; index++)
        {
            last[index] = *g_s4_watch_list[index].p_loops;
        }
        (void) tk_dly_tsk(1000u);
        for (uint8_t index = 0u; index < count; index++)
        {
            if (*g_s4_watch_list[index].p_loops == last[index])
            {
                T_RTSK info = {0};
                ER const error =
                    tk_ref_tsk(g_s4_watch_list[index].task_id, &info);

                tm_printf((UB *) "s4 watch: %s loop stalled (imu stage %u; "
                          "ref %d, stat 0x%x, wait 0x%lx, wid %d)\n",
                          g_s4_watch_list[index].p_name,
                          (UINT) g_s4_diag_stage, (INT) error,
                          (UINT) info.tskstat,
                          (UW) info.tskwait, (INT) info.wid);
            }
        }
    }
}
#endif

static s4_terrain_config_t
s4_runtime_config (void)
{
    s4_terrain_config_t config =
    {
        .calibration_samples = S4_CALIBRATION_SAMPLES,
        .calibration_span_mg = S4_CALIBRATION_SPAN_MG,
        .impact_threshold_mg = S4_IMPACT_THRESHOLD_MG,
        .max_sample_gap_ms = S4_MAX_SAMPLE_GAP_MS,
        .max_hump_ms = S4_MAX_HUMP_MS,
        .track_mm = S4_TRACK_MM,
        .wheelbase_mm = S4_WHEELBASE_MM,
        .rise_um = S4_RISE_UM,
        .level_um = S4_LEVEL_UM,
        .confirmation_samples = S4_CONFIRMATION_SAMPLES,
        .invalid_limit = S4_INVALID_SAMPLE_LIMIT,
        .average_samples = S4_AVERAGE_SAMPLES,
        .height_scale_permille = S4_HEIGHT_SCALE_PERMILLE,
        .lift_up_mg = S4_LIFT_UP_MG,
        .lift_tilt_mg = S4_LIFT_TILT_MG,
        .lift_samples = S4_LIFT_SAMPLES,
        .lift_raw_tilt_mg = S4_LIFT_RAW_TILT_MG,
        .lift_raw_samples = S4_LIFT_RAW_SAMPLES
    };

    return (config);
}

static uint32_t
s4_capture_time_us (void)
{
    SYSTIM current_time = {0};

    (void) tk_get_tim(&current_time);

    /* Unsigned wrap is intentional and handled by terrain_logic.c. */
    return ((uint32_t) current_time.lo * 1000u);
}

static void
s4_mark_sensor_error (s4_terrain_context_t * p_context,
                      uint32_t capture_time_us)
{
    s4_terrain_sample_t invalid_sample =
    {
        .capture_time_us = capture_time_us,
        .b_valid = false
    };

    for (uint8_t index = 0u;
         (index < S4_INVALID_SAMPLE_LIMIT) &&
         (S4_TERRAIN_STATE_FAULT != p_context->result.state);
         index++)
    {
        s4_terrain_process(p_context, &invalid_sample, false);
    }
}

/*
 * Reads Motion Control's mutex-protected snapshot (S2 owns the encoders).
 * A new run_id means the controller started a new run (START), so the run
 * maximum is cleared. Returns whether pitch may be read as tilt: false
 * while the mean wheel speed is changing (forward acceleration also reads
 * as pitch), for S4_PITCH_HOLD_SAMPLES after, and when the read fails.
 */
static bool
s4_motion_read (s4_motion_t * p_motion,
                s4_terrain_context_t * p_terrain)
{
    motion_status_t current;
    bool b_pitch_valid = false;

    if (E_OK != motion_get_status(&current))
    {
        p_motion->b_have_previous = false;
    }
    else
    {
        int32_t speed_mm_s = ((int32_t) current.left_mm_s +
                              (int32_t) current.right_mm_s) / 2;

        if (p_motion->b_have_previous)
        {
            int32_t change_mm_s = speed_mm_s - p_motion->speed_mm_s;

            if (current.run_id != p_motion->run_id)
            {
                s4_terrain_start_new_run(p_terrain);
                tm_printf((UB *) "s4: new run %u; hump maximum cleared\n",
                          (UINT) current.run_id);
            }

            if ((change_mm_s > S4_PITCH_DV_MM_S) ||
                (change_mm_s < -S4_PITCH_DV_MM_S))
            {
                p_motion->pitch_hold = S4_PITCH_HOLD_SAMPLES;
            }
            else if (p_motion->pitch_hold > 0u)
            {
                p_motion->pitch_hold--;
            }
            else
            {
                b_pitch_valid = true;
            }
        }

        p_motion->speed_mm_s = speed_mm_s;
        p_motion->run_id = current.run_id;
        p_motion->b_have_previous = true;
    }

    return (b_pitch_valid);
}

/*
 * Whether hump, impact and lift detection may run this sample (decisions
 * 12, 14 and 16): the robot is Running in FOLLOW or NAVIGATE, and has been
 * for S4_SETTLE_SAMPLES samples. BYPASS and REACQUIRE are excluded: their
 * fast turns read as lifts and humps (decision 16), as SCAN's braking
 * would. *p_settle counts down while driving and is reloaded whenever it
 * is not. A failed state read counts as not driving.
 */
static bool
s4_detect_gate (uint8_t * p_settle)
{
    s2_robot_state_t state;
    bool b_driving = false;
    bool b_enabled = false;

    if (E_OK == s2_get_robot_state(&state))
    {
        b_driving = (S2_SYS_RUNNING == state.state) &&
                    ((S2_MODE_FOLLOW == state.mode) ||
                     (S2_MODE_NAVIGATE == state.mode));
    }

    if (!b_driving)
    {
        *p_settle = S4_SETTLE_SAMPLES;
    }
    else if (*p_settle > 0u)
    {
        (*p_settle)--;
    }
    else
    {
        b_enabled = true;
    }

    return (b_enabled);
}

/*
 * Rotate a sample from the sensor's axes to the vehicle's (x forward,
 * y left, z up), which the terrain logic expects. As mounted (decision 7),
 * seen from behind with the ultrasonic sensor facing away, the board's X
 * points left, Y backwards and Z up. Negation cannot overflow: decoded
 * values lie within -2048..2047 mg.
 */
static void
s4_sample_to_vehicle (s4_terrain_sample_t * p_sample)
{
    int16_t sensor_x_mg = p_sample->acceleration_x_mg;
    int16_t sensor_y_mg = p_sample->acceleration_y_mg;

    /* -2047..2048 mg: negating a 12-bit value fits int16_t. */
    p_sample->acceleration_x_mg = (int16_t) -sensor_y_mg;
    p_sample->acceleration_y_mg = sensor_x_mg;
}

#if S4_RAW_LOG
/* Diagnostic (S4-H02, S4-H08): one line per sample, after processing.
 * Time in ms; acceleration in the vehicle's axes; the averaged tilts the
 * hump logic uses; whether pitch was read; the state; whether hump
 * detection was enabled. */
static void
s4_raw_log (s4_terrain_sample_t const * p_vehicle,
            s4_terrain_result_t const * p_result,
            bool b_pitch_valid,
            bool b_detect_enabled)
{
    tm_printf((UB *) "s4 raw %u: fwd=%d left=%d up=%d mg | tilt fwd=%d "
              "left=%d mg | pitch %s | state %u | detect %s\n",
              (UINT) (p_vehicle->capture_time_us / 1000u),
              (INT) p_vehicle->acceleration_x_mg,
              (INT) p_vehicle->acceleration_y_mg,
              (INT) p_vehicle->acceleration_z_mg,
              (INT) p_result->forward_tilt_mg,
              (INT) p_result->lateral_tilt_mg,
              b_pitch_valid ? "used" : "ignored",
              (UINT) p_result->state,
              b_detect_enabled ? "on" : "off");
}
#endif

static void
s4_report_events (s4_terrain_result_t const * p_result,
                  uint32_t calibration_start_us)
{
    if (0u != (p_result->events & S4_TERRAIN_EVENT_READY))
    {
        /* Unsigned subtraction is wrap-safe; the time measures the real
         * read rate for S4-H05 (10 ms timestamp resolution). */
        uint32_t elapsed_ms =
            (s4_capture_time_us() - calibration_start_us) / 1000u;

        tm_printf((UB *) "s4: stationary calibration complete "
                  "(%u samples in %u ms)\n",
                  (UINT) S4_CALIBRATION_SAMPLES, (UINT) elapsed_ms);
    }
    if (0u != (p_result->events & S4_TERRAIN_EVENT_HUMP_COMPLETE))
    {
        static char const * const s_contact_names[] =
        {
            "unknown", "left wheel", "right wheel", "both wheels", "caster"
        };
        char const * p_name = s_contact_names[0];

        if (p_result->hump_contact <= S4_HUMP_CONTACT_CASTER)
        {
            p_name = s_contact_names[p_result->hump_contact];
        }
        tm_printf((UB *) "s4: hump %u mm under the %s (run maximum %u mm)\n",
                  (UINT) p_result->current_height_mm, p_name,
                  (UINT) p_result->maximum_height_mm);
    }
    if (0u != (p_result->events & S4_TERRAIN_EVENT_IMPACT))
    {
        tm_printf((UB *) "s4: candidate impact detected\n");
    }
    if (0u != (p_result->events & S4_TERRAIN_EVENT_LIFT))
    {
        ID const flags = controller_flags;

        /* The Vehicle Controller latches STOP (IF-19, decision 15). The
         * flag is created by subsystem 2, so it may be absent. */
        if ((flags > 0) &&
            (tk_set_flg(flags, CTRL_FLG_PICKUP) >= E_OK))
        {
            tm_printf((UB *) "s4: car lifted; STOP requested\n");
        }
        else
        {
            tm_printf((UB *) "s4: car lifted; STOP request failed\n");
        }
    }
}

/* End of file. */
