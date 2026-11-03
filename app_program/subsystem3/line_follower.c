/**
 * @file line_follower.c
 * @brief Pure two-sensor line-position and fixed-point PD-control logic.
 *
 * Two sensors straddling a black line cannot distinguish an exactly centred
 * line from every possible lost-line geometry when both read white. Therefore,
 * normal both-white readings mean centred. The Vehicle Controller starts the
 * explicit recovery mode after a turn, bypass, or independently detected loss.
 */

#include "line_follower.h"

#include <limits.h>
#include <stddef.h>

static bool line_follower_config_is_valid(
    line_follower_config_t const * p_config);
static uint16_t line_follower_add_saturated(uint16_t value,
                                            uint16_t increment);
static int8_t line_follower_classify(line_follower_sample_t const * p_sample);
static void line_follower_update_candidate(line_follower_t * p_follower,
                                           int8_t observed_error);
static int16_t line_follower_calculate_correction(
    line_follower_t const * p_follower);
static uint16_t line_follower_limit_target(
    int32_t target,
    line_follower_config_t const * p_config);
static void line_follower_set_targets(line_follower_t const * p_follower,
                                      int16_t correction,
                                      line_follower_output_t * p_output);
static void line_follower_process_recovery(
    line_follower_t * p_follower,
    line_follower_sample_t const * p_sample,
    line_follower_output_t * p_output);
static void line_follower_process_junction(
    line_follower_t * p_follower,
    line_follower_sample_t const * p_sample,
    line_follower_output_t * p_output);

bool
line_follower_init (line_follower_t * p_follower,
                    line_follower_config_t const * p_config)
{
    bool b_initialized = false;

    if ((NULL != p_follower) && line_follower_config_is_valid(p_config))
    {
        p_follower->config                = *p_config;
        p_follower->state                 = LINE_FOLLOWER_STATE_FOLLOWING;
        p_follower->current_error         = 0;
        p_follower->previous_error        = 0;
        p_follower->candidate_error       = 0;
        p_follower->recovery_direction    = LINE_FOLLOWER_DIRECTION_RIGHT;
        p_follower->junction_distance_mm  = 0U;
        p_follower->candidate_count       = 0U;
        p_follower->junction_count        = 0U;
        p_follower->junction_clear_count  = 0U;
        p_follower->reacquire_count       = 0U;
        p_follower->pending_events        = LINE_FOLLOWER_EVENT_NONE;
        p_follower->b_junction_reported   = false;
        p_follower->b_recovery_edge_seen  = false;
        p_follower->b_initialized         = true;
        b_initialized                     = true;
    }

    return (b_initialized);
}

bool
line_follower_step (line_follower_t * p_follower,
                    line_follower_sample_t const * p_sample,
                    line_follower_output_t * p_output)
{
    bool b_processed = false;

    if ((NULL != p_follower) && (NULL != p_sample) &&
        (NULL != p_output) && p_follower->b_initialized)
    {
        p_output->events = p_follower->pending_events;
        p_follower->pending_events = LINE_FOLLOWER_EVENT_NONE;

        if (LINE_FOLLOWER_STATE_RECOVERING == p_follower->state)
        {
            line_follower_process_recovery(p_follower, p_sample, p_output);
        }
        else if (p_sample->b_left_black && p_sample->b_right_black)
        {
            line_follower_process_junction(p_follower, p_sample, p_output);
        }
        else
        {
            int8_t const observed_error = line_follower_classify(p_sample);
            int16_t correction = 0;

            p_follower->junction_count       = 0U;
            p_follower->junction_distance_mm = 0U;

            if (p_follower->b_junction_reported)
            {
                p_follower->junction_clear_count++;
                if (p_follower->junction_clear_count >=
                    p_follower->config.decision_samples)
                {
                    p_follower->b_junction_reported  = false;
                    p_follower->junction_clear_count = 0U;
                }
            }

            p_follower->state = LINE_FOLLOWER_STATE_FOLLOWING;
            line_follower_update_candidate(p_follower, observed_error);
            correction = line_follower_calculate_correction(p_follower);

            p_output->position = LINE_FOLLOWER_POSITION_CENTRED;
            if (0 < p_follower->current_error)
            {
                p_output->position = LINE_FOLLOWER_POSITION_ROBOT_LEFT;
            }
            else if (0 > p_follower->current_error)
            {
                p_output->position = LINE_FOLLOWER_POSITION_ROBOT_RIGHT;
            }
            else
            {
                /* The centred value is already selected. */
            }

            p_output->state        = p_follower->state;
            p_output->error        = p_follower->current_error;
            p_output->correction   = correction;
            p_output->b_line_valid = true;
            line_follower_set_targets(p_follower, correction, p_output);

            p_follower->previous_error = p_follower->current_error;
        }

        b_processed = true;
    }

    return (b_processed);
}

bool
line_follower_start_recovery (line_follower_t * p_follower,
                              line_follower_direction_t direction)
{
    bool b_started = false;

    if ((NULL != p_follower) && p_follower->b_initialized &&
        ((LINE_FOLLOWER_DIRECTION_LEFT == direction) ||
         (LINE_FOLLOWER_DIRECTION_RIGHT == direction)))
    {
        p_follower->state                = LINE_FOLLOWER_STATE_RECOVERING;
        p_follower->recovery_direction   = direction;
        p_follower->reacquire_count      = 0U;
        p_follower->b_recovery_edge_seen = false;
        p_follower->pending_events       = LINE_FOLLOWER_EVENT_LINE_LOST;
        b_started                        = true;
    }

    return (b_started);
}

static bool
line_follower_config_is_valid (line_follower_config_t const * p_config)
{
    bool b_valid = false;

    if ((NULL != p_config) &&
        (p_config->minimum_speed <= p_config->base_speed) &&
        (p_config->base_speed <= p_config->maximum_speed) &&
        (p_config->maximum_correction <= p_config->maximum_speed) &&
        (p_config->maximum_correction <= INT16_MAX) &&
        (0U < p_config->decision_samples) &&
        (0U < p_config->junction_samples) &&
        (0U < p_config->reacquire_samples))
    {
        b_valid = true;
    }

    return (b_valid);
}

static uint16_t
line_follower_add_saturated (uint16_t value, uint16_t increment)
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

static int8_t
line_follower_classify (line_follower_sample_t const * p_sample)
{
    int8_t error = 0;

    if (p_sample->b_right_black && (!p_sample->b_left_black))
    {
        error = 1;
    }
    else if (p_sample->b_left_black && (!p_sample->b_right_black))
    {
        error = -1;
    }
    else
    {
        /* Both white is centred; both black is handled as a junction. */
    }

    return (error);
}

static void
line_follower_update_candidate (line_follower_t * p_follower,
                                int8_t observed_error)
{
    if (p_follower->candidate_error == observed_error)
    {
        if (UINT8_MAX > p_follower->candidate_count)
        {
            p_follower->candidate_count++;
        }
    }
    else
    {
        p_follower->candidate_error = observed_error;
        p_follower->candidate_count = 1U;
    }

    if (p_follower->candidate_count >= p_follower->config.decision_samples)
    {
        p_follower->current_error = p_follower->candidate_error;
    }
}

static int16_t
line_follower_calculate_correction (line_follower_t const * p_follower)
{
    /* Bounded 8/16-bit values fit int32_t; casts force signed arithmetic. */
    int32_t const derivative = (int32_t)p_follower->current_error -
                               (int32_t)p_follower->previous_error;
    int32_t correction_q8 =
        ((int32_t)p_follower->config.proportional_gain_q8 *
         (int32_t)p_follower->current_error) +
        ((int32_t)p_follower->config.derivative_gain_q8 * derivative);
    int32_t correction = correction_q8 / (int32_t)LINE_FOLLOWER_GAIN_SCALE;
    int32_t const limit = (int32_t)p_follower->config.maximum_correction;

    if (limit < correction)
    {
        correction = limit;
    }
    else if (-limit > correction)
    {
        correction = -limit;
    }
    else
    {
        /* The correction is already inside the configured range. */
    }

    /* Safe because correction is limited to the configured uint16_t value. */
    return ((int16_t)correction);
}

static uint16_t
line_follower_limit_target (int32_t target,
                            line_follower_config_t const * p_config)
{
    /* Every uint16_t speed limit converts exactly to int32_t. */
    int32_t limited_target = target;

    if ((int32_t)p_config->minimum_speed > limited_target)
    {
        limited_target = (int32_t)p_config->minimum_speed;
    }
    else if ((int32_t)p_config->maximum_speed < limited_target)
    {
        limited_target = (int32_t)p_config->maximum_speed;
    }
    else
    {
        /* The requested target is already inside the configured range. */
    }

    /* Safe because the result is clamped to uint16_t configuration fields. */
    return ((uint16_t)limited_target);
}

static void
line_follower_set_targets (line_follower_t const * p_follower,
                           int16_t correction,
                           line_follower_output_t * p_output)
{
    /* The uint16_t base and int16_t correction both convert exactly. */
    int32_t const base_speed = (int32_t)p_follower->config.base_speed;

    p_output->left_target = line_follower_limit_target(
        base_speed + (int32_t)correction, &p_follower->config);
    p_output->right_target = line_follower_limit_target(
        base_speed - (int32_t)correction, &p_follower->config);
}

static void
line_follower_process_recovery (line_follower_t * p_follower,
                                line_follower_sample_t const * p_sample,
                                line_follower_output_t * p_output)
{
    int16_t correction = 0;

    if (p_sample->b_left_black != p_sample->b_right_black)
    {
        p_follower->b_recovery_edge_seen = true;
        p_follower->reacquire_count = 0U;
    }
    else if ((!p_sample->b_left_black) &&
             (!p_sample->b_right_black) &&
             p_follower->b_recovery_edge_seen)
    {
        if (UINT8_MAX > p_follower->reacquire_count)
        {
            p_follower->reacquire_count++;
        }

        if (p_follower->reacquire_count >=
            p_follower->config.reacquire_samples)
        {
            p_follower->state = LINE_FOLLOWER_STATE_FOLLOWING;
            p_follower->current_error  = 0;
            p_follower->previous_error = 0;
            p_output->events |= LINE_FOLLOWER_EVENT_REACQUIRED;
        }
    }
    else
    {
        p_follower->reacquire_count = 0U;
    }

    if (LINE_FOLLOWER_STATE_RECOVERING == p_follower->state)
    {
        /* Config validation ensures this signed product fits in int16_t. */
        correction = (int16_t)(
            (int32_t)p_follower->recovery_direction *
            (int32_t)p_follower->config.maximum_correction);
        p_output->position     = LINE_FOLLOWER_POSITION_UNKNOWN;
        p_output->error        = p_follower->recovery_direction;
        p_output->b_line_valid = false;
    }
    else
    {
        p_output->position     = LINE_FOLLOWER_POSITION_CENTRED;
        p_output->error        = 0;
        p_output->b_line_valid = true;
    }

    p_output->state      = p_follower->state;
    p_output->correction = correction;
    line_follower_set_targets(p_follower, correction, p_output);
}

static void
line_follower_process_junction (line_follower_t * p_follower,
                                line_follower_sample_t const * p_sample,
                                line_follower_output_t * p_output)
{
    if (UINT8_MAX > p_follower->junction_count)
    {
        p_follower->junction_count++;
    }

    p_follower->junction_distance_mm = line_follower_add_saturated(
        p_follower->junction_distance_mm, p_sample->travel_delta_mm);

    if ((!p_follower->b_junction_reported) &&
        (p_follower->junction_count >= p_follower->config.junction_samples) &&
        (p_follower->junction_distance_mm >=
         p_follower->config.junction_minimum_mm))
    {
        p_follower->state = LINE_FOLLOWER_STATE_JUNCTION;
        p_follower->b_junction_reported  = true;
        p_follower->junction_clear_count = 0U;
        p_output->events |= LINE_FOLLOWER_EVENT_JUNCTION;
    }

    p_output->position     = LINE_FOLLOWER_POSITION_UNKNOWN;
    p_output->state        = p_follower->state;
    p_output->error        = p_follower->current_error;
    p_output->correction   = 0;
    p_output->b_line_valid = true;
    line_follower_set_targets(p_follower, 0, p_output);
}

/* End of file. */
