/**
 * @file task_imu.c
 * @brief Periodic IMU sampling and terrain processing task.
 */
#include "subsystem4_internal.h"

#define S4_CALIBRATION_SAMPLES      200u
#define S4_CALIBRATION_SPAN_MG      80u
#define S4_ASCENT_THRESHOLD_MG      120u
#define S4_LEVEL_THRESHOLD_MG       50u
#define S4_IMPACT_THRESHOLD_MG      1200u
#define S4_MAX_SAMPLE_GAP_MS        50u
#define S4_CONFIRMATION_SAMPLES     4u
#define S4_INVALID_SAMPLE_LIMIT     3u

#ifndef S4_RAW_LOG
#define S4_RAW_LOG                  0
#endif
#define S4_RAW_LOG_INTERVAL         16u   /* samples, about 0.5 s */

static s4_terrain_config_t s4_runtime_config(void);
static uint32_t s4_capture_time_us(void);
static void s4_mark_sensor_error(s4_terrain_context_t * p_context,
                                 uint32_t capture_time_us);
static void s4_sample_to_vehicle(s4_terrain_sample_t * p_sample);
#if S4_RAW_LOG
static void s4_raw_log(s4_terrain_sample_t const * p_sensor,
                       s4_terrain_sample_t const * p_vehicle);
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

        error = s4_lsm303_read_acceleration(&driver,
                                            s4_capture_time_us(),
                                            &sample);
        if (E_OK == error)
        {
#if S4_RAW_LOG
            s4_terrain_sample_t sensor_sample = sample;

            s4_sample_to_vehicle(&sample);
            s4_raw_log(&sensor_sample, &sample);
#else
            s4_sample_to_vehicle(&sample);
#endif
            /* Encoder distance is deliberately unavailable pre-integration. */
            s4_terrain_process(&terrain, &sample, 0u, false);
            s4_report_events(&terrain.result, calibration_start_us);
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

        if (S4_TERRAIN_STATE_FAULT == terrain.result.state)
        {
            tm_printf((UB *) "s4: terrain fault %u; sensor restart\n",
                      (UINT) terrain.result.fault);
            (void) s4_lsm303_close(&driver);
            (void) tk_dly_tsk(S4_RETRY_PERIOD_MS);
        }
        else
        {
            (void) tk_dly_tsk(S4_POLL_PERIOD_MS);
        }
    }
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

    p_sample->acceleration_x_mg = (int16_t) -sensor_y_mg;
    p_sample->acceleration_y_mg = sensor_x_mg;
}

#if S4_RAW_LOG
/* S4-H02 diagnostic: one line every S4_RAW_LOG_INTERVAL samples. Called
 * only from s4_imu_task, so the static counter has a single user. */
static void
s4_raw_log (s4_terrain_sample_t const * p_sensor,
            s4_terrain_sample_t const * p_vehicle)
{
    static uint8_t s_count = 0u;

    s_count++;
    if (s_count >= S4_RAW_LOG_INTERVAL)
    {
        s_count = 0u;
        tm_printf((UB *) "s4 raw: sensor x=%d y=%d z=%d mg | vehicle "
                  "fwd=%d left=%d up=%d mg\n",
                  (INT) p_sensor->acceleration_x_mg,
                  (INT) p_sensor->acceleration_y_mg,
                  (INT) p_sensor->acceleration_z_mg,
                  (INT) p_vehicle->acceleration_x_mg,
                  (INT) p_vehicle->acceleration_y_mg,
                  (INT) p_vehicle->acceleration_z_mg);
    }
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
        tm_printf((UB *) "s4: hump detected; height unavailable without "
                  "odometry\n");
    }
    if (0u != (p_result->events & S4_TERRAIN_EVENT_IMPACT))
    {
        tm_printf((UB *) "s4: candidate impact detected\n");
    }
}

/* End of file. */
