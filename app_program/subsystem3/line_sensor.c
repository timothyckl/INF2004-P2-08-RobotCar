/**
 * @file line_sensor.c
 * @brief Active-object adapter for IR sampling and line-following logic.
 *
 * The default gains, period, black polarity, and confirmation distances are
 * initial tuning values only. They are not validated hardware results.
 */

#include "line_sensor.h"

#include "drv_ir_line.h"

#include <limits.h>

#define LINE_SENSOR_PERIOD_MSEC          (10U)
#define LINE_SENSOR_MINIMUM_SPEED        (120U)
#define LINE_SENSOR_BASE_SPEED           (250U)
#define LINE_SENSOR_MAXIMUM_SPEED        (380U)
#define LINE_SENSOR_MAXIMUM_CORRECTION   (130U)
#define LINE_SENSOR_PROPORTIONAL_GAIN_Q8 (20480U)
#define LINE_SENSOR_DERIVATIVE_GAIN_Q8   (7680U)
#define LINE_SENSOR_DECISION_SAMPLES     (2U)
#define LINE_SENSOR_JUNCTION_SAMPLES     (3U)
#define LINE_SENSOR_JUNCTION_MINIMUM_MM  (2U)
#define LINE_SENSOR_REACQUIRE_SAMPLES    (3U)

typedef struct
{
    uint16_t                  pending_travel_mm;
    line_follower_direction_t recovery_direction;
    bool                      b_left_black_level;
    bool                      b_right_black_level;
    bool                      b_recovery_requested;
} line_sensor_command_t;

static ID                     g_h_line_sensor_mutex = 0;
static line_sensor_snapshot_t g_line_sensor_snapshot = {0};
static line_sensor_command_t  g_line_sensor_command = {0};
static bool                   g_b_line_sensor_initialized = false;

static void line_sensor_task(INT start_code, void * p_extension);
static uint16_t line_sensor_add_saturated(uint16_t value,
                                          uint16_t increment);
static ER line_sensor_publish_sample(
    drv_ir_line_sample_t const * p_raw_sample,
    line_follower_sample_t const * p_sample,
    line_follower_output_t const * p_output);
static void line_sensor_publish_fault(void);

ER
line_sensor_init (void)
{
    T_CMTX mutex_config = {0};
    ER error = E_OBJ;

    if (!g_b_line_sensor_initialized)
    {
        mutex_config.mtxatr = TA_INHERIT;
        g_h_line_sensor_mutex = tk_cre_mtx(&mutex_config);
        if (E_OK > g_h_line_sensor_mutex)
        {
            error = g_h_line_sensor_mutex;
        }
        else
        {
            g_line_sensor_command.b_left_black_level  = false;
            g_line_sensor_command.b_right_black_level = false;
            g_line_sensor_command.recovery_direction =
                LINE_FOLLOWER_DIRECTION_RIGHT;
            g_b_line_sensor_initialized = true;
            error = E_OK;
        }
    }

    return (error);
}

ER
line_sensor_start (void)
{
    ER error = E_OBJ;

    if (g_b_line_sensor_initialized)
    {
        /* The RTOS stores task entry points in its generic FP function type. */
        ID const task_id = app_start_task((FP)line_sensor_task,
                                          LINE_SENSOR_TASK_PRIORITY,
                                          TP_PRC1,
                                          "line_sensor_task");
        error = E_OK;

        if (E_OK > task_id)
        {
            error = task_id;
        }
    }

    return (error);
}

ER
line_sensor_get_snapshot (line_sensor_snapshot_t * p_snapshot)
{
    ER error = E_PAR;

    if (!g_b_line_sensor_initialized)
    {
        error = E_OBJ;
    }
    else if (NULL != p_snapshot)
    {
        error = tk_loc_mtx(g_h_line_sensor_mutex, TMO_FEVR);
        if (E_OK == error)
        {
            *p_snapshot = g_line_sensor_snapshot;
            error = tk_unl_mtx(g_h_line_sensor_mutex);
        }
    }

    return (error);
}

ER
line_sensor_take_events (line_follower_event_t * p_events)
{
    ER error = E_PAR;

    if (!g_b_line_sensor_initialized)
    {
        error = E_OBJ;
    }
    else if (NULL != p_events)
    {
        error = tk_loc_mtx(g_h_line_sensor_mutex, TMO_FEVR);
        if (E_OK == error)
        {
            *p_events = g_line_sensor_snapshot.latched_events;
            g_line_sensor_snapshot.latched_events =
                LINE_FOLLOWER_EVENT_NONE;
            error = tk_unl_mtx(g_h_line_sensor_mutex);
        }
    }

    return (error);
}

ER
line_sensor_set_calibration (bool b_left_black_level,
                             bool b_right_black_level)
{
    ER error = E_OBJ;

    if (g_b_line_sensor_initialized)
    {
        error = tk_loc_mtx(g_h_line_sensor_mutex, TMO_FEVR);
        if (E_OK == error)
        {
            g_line_sensor_command.b_left_black_level = b_left_black_level;
            g_line_sensor_command.b_right_black_level = b_right_black_level;
            error = tk_unl_mtx(g_h_line_sensor_mutex);
        }
    }

    return (error);
}

ER
line_sensor_add_travel_mm (uint16_t travel_delta_mm)
{
    ER error = E_OBJ;

    if (g_b_line_sensor_initialized)
    {
        error = tk_loc_mtx(g_h_line_sensor_mutex, TMO_FEVR);
        if (E_OK == error)
        {
            g_line_sensor_command.pending_travel_mm =
                line_sensor_add_saturated(
                    g_line_sensor_command.pending_travel_mm,
                    travel_delta_mm);
            error = tk_unl_mtx(g_h_line_sensor_mutex);
        }
    }

    return (error);
}

ER
line_sensor_start_recovery (line_follower_direction_t direction)
{
    ER error = E_PAR;

    if (!g_b_line_sensor_initialized)
    {
        error = E_OBJ;
    }
    else if ((LINE_FOLLOWER_DIRECTION_LEFT == direction) ||
        (LINE_FOLLOWER_DIRECTION_RIGHT == direction))
    {
        error = tk_loc_mtx(g_h_line_sensor_mutex, TMO_FEVR);
        if (E_OK == error)
        {
            g_line_sensor_command.recovery_direction = direction;
            g_line_sensor_command.b_recovery_requested = true;
            error = tk_unl_mtx(g_h_line_sensor_mutex);
        }
    }

    return (error);
}

static void
line_sensor_task (INT start_code, void * p_extension)
{
    line_follower_config_t const config =
    {
        .minimum_speed        = LINE_SENSOR_MINIMUM_SPEED,
        .base_speed           = LINE_SENSOR_BASE_SPEED,
        .maximum_speed        = LINE_SENSOR_MAXIMUM_SPEED,
        .maximum_correction   = LINE_SENSOR_MAXIMUM_CORRECTION,
        .proportional_gain_q8 = LINE_SENSOR_PROPORTIONAL_GAIN_Q8,
        .derivative_gain_q8   = LINE_SENSOR_DERIVATIVE_GAIN_Q8,
        .junction_minimum_mm  = LINE_SENSOR_JUNCTION_MINIMUM_MM,
        .decision_samples     = LINE_SENSOR_DECISION_SAMPLES,
        .junction_samples     = LINE_SENSOR_JUNCTION_SAMPLES,
        .reacquire_samples    = LINE_SENSOR_REACQUIRE_SAMPLES
    };
    line_follower_t follower = {0};
    ER error = E_OK;

    /* These parameters are required by the micro T-Kernel task ABI. */
    (void)start_code;
    (void)p_extension;

    error = drv_ir_line_init();
    if ((E_OK != error) || (!line_follower_init(&follower, &config)))
    {
        line_sensor_publish_fault();
    }

    for (;;)
    {
        drv_ir_line_sample_t raw_sample = {0};
        line_sensor_command_t command = {0};
        line_follower_sample_t sample = {0};
        line_follower_output_t output = {0};

        if (E_OK == error)
        {
            error = drv_ir_line_read(&raw_sample);
        }

        if (E_OK == error)
        {
            error = tk_loc_mtx(g_h_line_sensor_mutex, TMO_FEVR);
        }

        if (E_OK == error)
        {
            command = g_line_sensor_command;
            g_line_sensor_command.pending_travel_mm = 0U;
            g_line_sensor_command.b_recovery_requested = false;
            error = tk_unl_mtx(g_h_line_sensor_mutex);
        }

        if (E_OK == error)
        {
            sample.b_left_black =
                (raw_sample.b_left_raw == command.b_left_black_level);
            sample.b_right_black =
                (raw_sample.b_right_raw == command.b_right_black_level);
            sample.travel_delta_mm = command.pending_travel_mm;

            if (command.b_recovery_requested)
            {
                if (!line_follower_start_recovery(
                        &follower, command.recovery_direction))
                {
                    error = E_PAR;
                }
            }

            if ((E_OK == error) &&
                (!line_follower_step(&follower, &sample, &output)))
            {
                error = E_SYS;
            }
        }

        if (E_OK == error)
        {
            error = line_sensor_publish_sample(&raw_sample, &sample, &output);
        }

        if (E_OK != error)
        {
            line_sensor_publish_fault();
        }

        tk_dly_tsk(LINE_SENSOR_PERIOD_MSEC);
    }
}

static uint16_t
line_sensor_add_saturated (uint16_t value, uint16_t increment)
{
    /* Both uint16_t operands fit in uint32_t, so this sum cannot overflow. */
    uint32_t const sum = (uint32_t)value + (uint32_t)increment;
    uint16_t result = UINT16_MAX;

    if (UINT16_MAX >= sum)
    {
        /* Safe because the preceding comparison bounds sum to UINT16_MAX. */
        result = (uint16_t)sum;
    }

    return (result);
}

static ER
line_sensor_publish_sample (drv_ir_line_sample_t const * p_raw_sample,
                            line_follower_sample_t const * p_sample,
                            line_follower_output_t const * p_output)
{
    ER error = tk_loc_mtx(g_h_line_sensor_mutex, TMO_FEVR);

    if (E_OK == error)
    {
        g_line_sensor_snapshot.control = *p_output;
        g_line_sensor_snapshot.sample_count++;
        g_line_sensor_snapshot.latched_events |= p_output->events;
        g_line_sensor_snapshot.b_left_raw = p_raw_sample->b_left_raw;
        g_line_sensor_snapshot.b_right_raw = p_raw_sample->b_right_raw;
        g_line_sensor_snapshot.b_left_black = p_sample->b_left_black;
        g_line_sensor_snapshot.b_right_black = p_sample->b_right_black;
        g_line_sensor_snapshot.b_sensor_fault = false;
        error = tk_unl_mtx(g_h_line_sensor_mutex);
    }

    return (error);
}

static void
line_sensor_publish_fault (void)
{
    if (E_OK == tk_loc_mtx(g_h_line_sensor_mutex, TMO_FEVR))
    {
        g_line_sensor_snapshot.b_sensor_fault = true;
        g_line_sensor_snapshot.control.position =
            LINE_FOLLOWER_POSITION_UNKNOWN;
        g_line_sensor_snapshot.control.state =
            LINE_FOLLOWER_STATE_SENSOR_FAULT;
        g_line_sensor_snapshot.control.events =
            LINE_FOLLOWER_EVENT_SENSOR_FAULT;
        g_line_sensor_snapshot.control.b_line_valid = false;
        g_line_sensor_snapshot.control.correction = 0;
        g_line_sensor_snapshot.control.left_target = 0U;
        g_line_sensor_snapshot.control.right_target = 0U;
        g_line_sensor_snapshot.latched_events |=
            LINE_FOLLOWER_EVENT_SENSOR_FAULT;
        tk_unl_mtx(g_h_line_sensor_mutex);
    }
}

/* End of file. */
