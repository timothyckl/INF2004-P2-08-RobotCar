/**
 * @file task_imu.c
 * @brief Periodic IMU sampling and terrain processing task.
 */
#include "subsystem4_internal.h"
#include "s4_config.h"
#include "../subsystem2/motion_api.h"   /* motion_get_status (IF-17) */
#include "../subsystem2/subsystem2.h"   /* s2_get_robot_state (IF-14) */
#include "../subsystem2/task_health.h"  /* liveness (S2 decision 37) */
#include "../subsystem2/motion_config.h" /* MC_STILL_MM_S */
#include "s4_motion_class.h"

/* The classifier's stillness limit must stay Motion Control's (decision
 * 20): a compile-time check, as s4_config.h cannot include motion_config.h
 * on the host. */
typedef char s4_still_matches_mc_t
    [(S4_MOTION_STILL_MM_S == MC_STILL_MM_S) ? 1 : -1];


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

static uint32_t s4_capture_time_us(void);
static void s4_mark_sensor_error(s4_terrain_context_t * p_context,
                                 uint32_t capture_time_us);
static void s4_sample_to_vehicle(s4_terrain_sample_t * p_sample);

/* Motion Control snapshot (IF-17): the mean-speed gate across samples,
   and this sample's wheel speeds for the motion classifier. */
typedef struct
{
    s4_speed_gate_t gate;
    uint32_t        age_ms;
    int16_t         left_mm_s;
    int16_t         right_mm_s;
    uint8_t         run_id;
    bool            b_have_run;
    bool            b_valid;    /* this sample's snapshot was read */
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
static void s4_classify(s4_motion_classifier_t * p_class,
                        s4_motion_t const * p_motion,
                        s4_terrain_context_t * p_terrain);

void
s4_imu_task (INT start_code,
             void * p_exinf)
{
    s4_lsm303_driver_t driver;
    s4_terrain_context_t terrain;
    s4_terrain_config_t config = s4_config_mission();
    s4_terrain_sample_t sample = {0};
    uint32_t calibration_start_us = 0u;
    s4_motion_t motion = {0};
    s4_motion_classifier_t motion_class;
    bool b_pitch_valid = false;
    bool b_detect_enabled = false;
    uint8_t settle = S4_SETTLE_SAMPLES;
    ER error;

    (void) start_code;
    (void) p_exinf;

    s4_lsm303_init(&driver);
    s4_terrain_init(&terrain, &config);
    s4_motion_class_init(&motion_class);
    (void) s4_status_publish(&terrain.result);

    for (;;)
    {
        /* Every pass, a sensor retry included (S2 decision 37). */
        s2_health_check_in(S2_HEALTH_IMU);

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
            s4_classify(&motion_class, &motion, &terrain);
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

                /* Console arguments: 8- to 32-bit integer values go to UINT
                   (unsigned) or INT (signed); both are 32-bit on the RP2040,
                   so no value changes. */
                tm_printf((UB *) "s4 watch: %s loop stalled (imu stage %u; "
                          "ref %d, stat 0x%x, wait 0x%x, wid %d)\n",
                          g_s4_watch_list[index].p_name,
                          (UINT) g_s4_diag_stage, (INT) error,
                          (UINT) info.tskstat,
                          (UINT) info.tskwait, (INT) info.wid);
            }
        }
    }
}
#endif

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

    p_context->result.motion_class = S4_MOTION_UNKNOWN;
    p_context->result.yaw_rate_ddeg_s = 0;

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
    SYSTIM now = {0};
    bool b_pitch_valid = false;

    p_motion->b_valid = (E_OK == motion_get_status(&current))
                        && (E_OK == tk_get_tim(&now));
    if (!p_motion->b_valid)
    {
        s4_speed_gate_reset(&p_motion->gate);
    }
    else
    {
        /* Widening: the wheel speeds are int16_t, so their sum fits int32_t
           without overflow and the mean lies between them. */
        int32_t const speed_mm_s = ((int32_t) current.left_mm_s +
                                    (int32_t) current.right_mm_s) / 2;

        if (p_motion->b_have_run && (current.run_id != p_motion->run_id))
        {
            s4_terrain_start_new_run(p_terrain);
            /* run_id is uint8_t, which UINT holds. */
            tm_printf((UB *) "s4: new run %u; hump maximum cleared\n",
                      (UINT) current.run_id);
        }

        b_pitch_valid = s4_speed_gate_step(&p_motion->gate, speed_mm_s,
                                           S4_PITCH_DV_MM_S,
                                           S4_PITCH_HOLD_SAMPLES);
        /* now.lo is the 32-bit ms word (UW): the modulo 2^32 difference
           is the snapshot's age, wrap-safe. */
        p_motion->age_ms = (uint32_t) now.lo - current.captured_ms;
        p_motion->left_mm_s = current.left_mm_s;
        p_motion->right_mm_s = current.right_mm_s;
        p_motion->run_id = current.run_id;
        p_motion->b_have_run = true;
    }

    return (b_pitch_valid);
}

/*
 * Motion classification (decision 20), observational only: one step per
 * sample, after the terrain logic; the state and the encoder yaw rate go
 * into the published result (IF-18) and a change is printed.
 */
static void
s4_classify (s4_motion_classifier_t * p_class,
             s4_motion_t const * p_motion,
             s4_terrain_context_t * p_terrain)
{
    s4_motion_class_config_t const config = s4_motion_config_mission();
    s4_terrain_result_t * const p_result = &p_terrain->result;
    s4_motion_class_t const previous = p_class->state;
    s4_motion_input_t const input =
    {
        .left_mm_s = p_motion->left_mm_s,
        .right_mm_s = p_motion->right_mm_s,
        .age_ms = p_motion->age_ms,
        .b_motion_valid = p_motion->b_valid,
        .speed_change = p_motion->gate.direction,
        .b_terrain_valid =
            (S4_TERRAIN_STATE_MONITORING == p_result->state) ||
            (S4_TERRAIN_STATE_ON_HUMP == p_result->state),
        .b_on_hump = (S4_TERRAIN_STATE_ON_HUMP == p_result->state),
        .height_um = p_terrain->height_um,
        .peak_um = p_terrain->peak_height_um,
        .b_impact = (0u != (p_result->events & S4_TERRAIN_EVENT_IMPACT))
    };
    s4_motion_class_t const state =
        s4_motion_class_step(p_class, &config, &input);

    p_result->motion_class = state;
    p_result->yaw_rate_ddeg_s = p_class->yaw_ddeg_s;
    if (state != previous)
    {
        /* Widening: yaw is int16_t, so its magnitude fits int32_t and,
           at most 32768, UINT. */
        int32_t const yaw = p_class->yaw_ddeg_s;
        int32_t const size = (yaw < 0) ? -yaw : yaw;

        tm_printf((UB *) "s4: motion %s (encoder yaw %s%u.%u deg/s)\n",
                  s4_motion_class_name(state), (yaw < 0) ? "-" : "",
                  (UINT) (size / 10), (UINT) (size % 10));
    }
}

/*
 * Whether hump, impact and lift detection may run this sample (decisions
 * 12, 14, 16 and 17): the robot is Running in FOLLOW, and has been for
 * S4_SETTLE_SAMPLES samples. BYPASS and REACQUIRE are excluded: their fast
 * turns read as lifts and humps (decision 16), as SCAN's braking would.
 * NAVIGATE is excluded too (decision 17): the junction stop and step read
 * up to 6.6 mm and the spin 4.7 mm in the 2026-11-17 turns, against at
 * most 4.0 mm while following. *p_settle counts down while driving and is
 * reloaded whenever it is not. A failed state read counts as not driving.
 */
static bool
s4_detect_gate (uint8_t * p_settle)
{
    s2_robot_state_t state;
    bool b_driving = false;

    if (E_OK == s2_get_robot_state(&state))
    {
        b_driving = (S2_SYS_RUNNING == state.state) &&
                    (S2_MODE_FOLLOW == state.mode);
    }

    return (s4_terrain_gate_step(p_settle, b_driving, S4_SETTLE_SAMPLES));
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
