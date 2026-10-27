/**
 * @file task_imu.c
 * @brief Periodic standalone IMU sampling and terrain processing task.
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

static s4_terrain_config_t s4_runtime_config(void);
static uint32_t s4_capture_time_us(void);
static void s4_mark_sensor_error(s4_terrain_context_t * p_context,
                                 uint32_t capture_time_us);
static void s4_report_events(s4_terrain_result_t const * p_result);

void
s4_imu_task (INT start_code,
             void * p_exinf)
{
    s4_lsm303_driver_t driver;
    s4_terrain_context_t terrain;
    s4_terrain_config_t config = s4_runtime_config();
    s4_terrain_sample_t sample = {0};
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
        }

        error = s4_lsm303_read_acceleration(&driver,
                                            s4_capture_time_us(),
                                            &sample);
        if (E_OK == error)
        {
            /* Encoder distance is deliberately unavailable pre-integration. */
            s4_terrain_process(&terrain, &sample, 0u, false);
            s4_report_events(&terrain.result);
            (void) s4_status_publish(&terrain.result);
        }
        else if (E_BUSY != error)
        {
            s4_mark_sensor_error(&terrain, s4_capture_time_us());
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
            (void) s4_lsm303_close(&driver);
            (void) tk_dly_tsk(S4_RETRY_PERIOD_MS);
        }
        else
        {
            (void) tk_dly_tsk(S4_SAMPLE_PERIOD_MS);
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

static void
s4_report_events (s4_terrain_result_t const * p_result)
{
    if (0u != (p_result->events & S4_TERRAIN_EVENT_READY))
    {
        tm_printf((UB *) "s4: stationary calibration complete\n");
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
