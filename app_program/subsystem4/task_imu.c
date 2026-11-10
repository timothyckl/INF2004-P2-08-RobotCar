/**
 * @file task_imu.c
 * @brief Periodic IMU sampling and terrain processing task.
 *
 * Encoder distance comes from subsystem 2's Motion Control status
 * (s4_odometry.c); without a valid, time-aligned distance the hump height
 * stays invalid, as before.
 */
#include "subsystem4_internal.h"
#include "s4_odometry.h"

#define S4_CALIBRATION_SAMPLES      200u
#define S4_CALIBRATION_SPAN_MG      80u
#define S4_ASCENT_THRESHOLD_MG      120u
#define S4_LEVEL_THRESHOLD_MG       50u
#define S4_IMPACT_THRESHOLD_MG      1200u
#define S4_MAX_SAMPLE_GAP_MS        50u
#define S4_CONFIRMATION_SAMPLES     4u
#define S4_INVALID_SAMPLE_LIMIT     3u

/* Samples discarded after each open: turn-on time is 7/ODR in normal and
   high-resolution mode (LSM303DLHC datasheet, Table 8). */
#define S4_TURN_ON_SAMPLES          7u

static s4_terrain_config_t s4_runtime_config(void);
static uint32_t s4_capture_time_us(void);
static void s4_mark_sensor_error(s4_terrain_context_t * p_context,
                                 uint32_t capture_time_us);
static void s4_mark_invalid_read(s4_terrain_context_t * p_context,
                                 uint32_t capture_time_us);
static void s4_report_events(s4_terrain_result_t const * p_result);
static void s4_retry_wait(void);
static void s4_process_sample(s4_terrain_context_t * p_terrain,
                              s4_odometry_t * p_odometry,
                              s4_terrain_sample_t const * p_sample);

void
s4_imu_task (INT start_code,
             void * p_exinf)
{
    s4_lsm303_driver_t driver;
    s4_terrain_context_t terrain;
    s4_terrain_config_t config = s4_runtime_config();
    s4_terrain_sample_t sample = {0};
    s4_odometry_t odometry;
    ER error;
    uint8_t turn_on_count = 0u;

    (void) start_code;
    (void) p_exinf;

    s4_odometry_init(&odometry);
    s4_lsm303_init(&driver);
    s4_terrain_init(&terrain, &config);
    (void) s4_status_publish(&terrain.result);

    for (;;)
    {
        if (!driver.b_open)
        {
            s4_terrain_request_recovery(&terrain);
            error = s4_lsm303_open(&driver);
            if (error < E_OK)
            {
                tm_printf((UB *) "s4: IMU startup failed (%d), retrying\n",
                          error);
                s4_mark_sensor_error(&terrain, s4_capture_time_us());
                (void) s4_status_publish(&terrain.result);
                s4_retry_wait();
                continue;
            }

            tm_printf((UB *) "s4: LSM303DLHC ready; calibrating\n");
            turn_on_count = S4_TURN_ON_SAMPLES;
        }

        error = s4_lsm303_read_acceleration(&driver,
                                            s4_capture_time_us(),
                                            &sample);
        if ((E_OK == error) && (turn_on_count > 0u))
        {
            turn_on_count--;    /* still settling: not a usable sample */
        }
        else if (E_OK == error)
        {
            s4_process_sample(&terrain, &odometry, &sample);
            s4_report_events(&terrain.result);
            (void) s4_status_publish(&terrain.result);
        }
        else if (E_BUSY != error)
        {
            /* One failed read is one invalid sample: the terrain logic
               faults after S4_INVALID_SAMPLE_LIMIT in a row. */
            s4_mark_invalid_read(&terrain, s4_capture_time_us());
            (void) s4_status_publish(&terrain.result);
        }
        else
        {
            /* The next 100 Hz period will poll data-ready again. */
        }

        if (S4_TERRAIN_STATE_FAULT == terrain.result.state)
        {
            tm_printf((UB *) "s4: terrain fault %u; sensor restart\n",
                      (UINT) terrain.result.fault);
            if (S4_TERRAIN_FAULT_CALIBRATION == terrain.result.fault)
            {
                /* Measured stationary spread, for S4-H03. */
                tm_printf((UB *) "s4: calibration span x %d y %d z %d mg"
                          " (limit %u) after %u samples\n",
                          (INT) (terrain.calibration_max_x -
                                 terrain.calibration_min_x),
                          (INT) (terrain.calibration_max_y -
                                 terrain.calibration_min_y),
                          (INT) (terrain.calibration_max_z -
                                 terrain.calibration_min_z),
                          (UINT) S4_CALIBRATION_SPAN_MG,
                          (UINT) terrain.calibration_count);
            }
            (void) s4_lsm303_close(&driver);
            s4_retry_wait();
        }
        else
        {
            /* Woken every S4_SAMPLE_PERIOD_MS by subsystem4.c. */
            (void) tk_slp_tsk(TMO_FEVR);
        }
    }
}

static void
s4_process_sample (s4_terrain_context_t * p_terrain,
                   s4_odometry_t * p_odometry,
                   s4_terrain_sample_t const * p_sample)
{
    motion_status_t status;
    s4_odometry_step_t step;
    SYSTIM now = {0};
    bool const b_status = (E_OK == motion_get_status(&status));

    (void) tk_get_tim(&now);
    s4_odometry_step(p_odometry, b_status ? &status : NULL,
                     (uint32_t) now.lo, &step);

    if (step.b_new_run)
    {
        s4_terrain_start_new_run(p_terrain);
    }

    s4_terrain_process(p_terrain, p_sample, step.delta_mm, step.b_valid);
}

static s4_terrain_config_t
s4_runtime_config (void)
{
    s4_terrain_config_t config =
    {
        .calibration_samples = S4_CALIBRATION_SAMPLES,
        .calibration_span_mg = S4_CALIBRATION_SPAN_MG,
        .ascent_threshold_mg = S4_ASCENT_THRESHOLD_MG,
        .level_threshold_mg = S4_LEVEL_THRESHOLD_MG,
        .impact_threshold_mg = S4_IMPACT_THRESHOLD_MG,
        .max_sample_gap_ms = S4_MAX_SAMPLE_GAP_MS,
        .confirmation_samples = S4_CONFIRMATION_SAMPLES,
        .invalid_limit = S4_INVALID_SAMPLE_LIMIT
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
        s4_terrain_process(p_context, &invalid_sample, 0u, false);
    }
}

static void
s4_mark_invalid_read (s4_terrain_context_t * p_context,
                      uint32_t capture_time_us)
{
    s4_terrain_sample_t invalid_sample =
    {
        .capture_time_us = capture_time_us,
        .b_valid = false
    };

    s4_terrain_process(p_context, &invalid_sample, 0u, false);
}

/* Waits before reopening the sensor, then discards the sample wakeups
   queued meanwhile so that sampling resumes at the normal period. */
static void
s4_retry_wait (void)
{
    (void) tk_dly_tsk(S4_RETRY_PERIOD_MS);
    (void) tk_can_wup(TSK_SELF);
}

static void
s4_report_events (s4_terrain_result_t const * p_result)
{
    if (0u != (p_result->events & S4_TERRAIN_EVENT_READY))
    {
        tm_printf((UB *) "s4: stationary calibration complete\n");
    }
    if (0u != (p_result->events & S4_TERRAIN_EVENT_HUMP_COMPLETE))
    {
        if (p_result->b_maximum_height_valid)
        {
            tm_printf((UB *) "s4: hump complete; run maximum %u mm "
                      "(provisional estimate)\n",
                      (UINT) p_result->maximum_height_mm);
        }
        else
        {
            tm_printf((UB *) "s4: hump complete; height invalid "
                      "(no valid distance)\n");
        }
    }
    if (0u != (p_result->events & S4_TERRAIN_EVENT_IMPACT))
    {
        tm_printf((UB *) "s4: candidate impact detected\n");
    }
}

/* End of file. */
