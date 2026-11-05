/**
 * @file pid.c
 * @brief Subsystem 2 integer PID controller (see pid.h).
 *
 * Products are formed in int64_t: |error| <= 2 * PID_INPUT_MAX and
 * |gain| <= PID_GAIN_MAX keep every term below 2^42. The integral is
 * clamped to the output range, and it stops growing while the output is
 * saturated in the direction of the error (conditional integration), so
 * it cannot wind up while a wheel is stalled. The derivative acts on the
 * measurement, not the error, so a setpoint step does not kick the output.
 */
#include "pid.h"

#include <stddef.h>

static bool    pid_cfg_valid (pid_cfg_t const * p_cfg);
static bool    pid_input_valid (int32_t value);
static int64_t pid_clamp (int64_t value, int64_t min, int64_t max);

/**
 * @brief Configure a controller and clear its state.
 * @return false, leaving *p_pid unchanged, if any argument is invalid.
 */
bool
pid_init (pid_ctrl_t * p_pid, pid_cfg_t const * p_cfg)
{
    if ((NULL == p_pid) || (!pid_cfg_valid(p_cfg)))
    {
        return (false);
    }

    p_pid->cfg = *p_cfg;
    pid_reset(p_pid);

    return (true);
}

/**
 * @brief Clear the integral and derivative history, keeping the tuning.
 *
 * Call whenever the loop restarts (e.g. after STOP), so stale history
 * does not drive the motors.
 */
void
pid_reset (pid_ctrl_t * p_pid)
{
    if (NULL != p_pid)
    {
        p_pid->i_sum      = 0;
        p_pid->prev_meas  = 0;
        p_pid->b_has_prev = false;
    }
}

/**
 * @brief Run one control step.
 * @return false, leaving state and *p_output unchanged, if an argument is
 *         NULL or an input exceeds PID_INPUT_MAX.
 */
bool
pid_update (pid_ctrl_t * p_pid, int32_t setpoint, int32_t measurement,
            int32_t * p_output)
{
    if ((NULL == p_pid) || (NULL == p_output)
        || (!pid_input_valid(setpoint)) || (!pid_input_valid(measurement)))
    {
        return (false);
    }

    pid_cfg_t const * p_cfg = &p_pid->cfg;
    int64_t const i_min = (int64_t)p_cfg->out_min * PID_GAIN_SCALE;
    int64_t const i_max = (int64_t)p_cfg->out_max * PID_GAIN_SCALE;
    int32_t const error = setpoint - measurement;
    int64_t const p_term = (int64_t)p_cfg->kp * error;
    int64_t       d_term = 0;
    int64_t       i_next = pid_clamp(p_pid->i_sum + ((int64_t)p_cfg->ki * error),
                                     i_min, i_max);

    if (p_pid->b_has_prev)
    {
        d_term = -((int64_t)p_cfg->kd * (measurement - p_pid->prev_meas));
    }

    /* Division truncates toward zero for either sign (C99 6.5.5). */
    int64_t output = (p_term + i_next + d_term) / PID_GAIN_SCALE;

    if (output >= p_cfg->out_max)
    {
        output = p_cfg->out_max;

        if (error > 0)
        {
            i_next = p_pid->i_sum;
        }
    }
    else if (output <= p_cfg->out_min)
    {
        output = p_cfg->out_min;

        if (error < 0)
        {
            i_next = p_pid->i_sum;
        }
    }
    else
    {
        /* Output within limits: keep the new integral. */
    }

    p_pid->i_sum      = i_next;
    p_pid->prev_meas  = measurement;
    p_pid->b_has_prev = true;
    *p_output         = (int32_t)output;

    return (true);
}

static bool
pid_cfg_valid (pid_cfg_t const * p_cfg)
{
    return ((NULL != p_cfg)
            && (p_cfg->kp >= 0) && (p_cfg->kp <= PID_GAIN_MAX)
            && (p_cfg->ki >= 0) && (p_cfg->ki <= PID_GAIN_MAX)
            && (p_cfg->kd >= 0) && (p_cfg->kd <= PID_GAIN_MAX)
            && (p_cfg->out_min < p_cfg->out_max));
}

static bool
pid_input_valid (int32_t value)
{
    return ((value >= -PID_INPUT_MAX) && (value <= PID_INPUT_MAX));
}

static int64_t
pid_clamp (int64_t value, int64_t min, int64_t max)
{
    int64_t result = value;

    if (value < min)
    {
        result = min;
    }
    else if (value > max)
    {
        result = max;
    }
    else
    {
        /* Already within range. */
    }

    return (result);
}

/* End of file. */
