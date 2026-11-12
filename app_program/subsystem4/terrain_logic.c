/**
 * @file terrain_logic.c
 * @brief Deterministic IMU calibration and terrain state machine.
 */
#include "terrain_logic.h"

#include <limits.h>
#include <stddef.h>

#define S4_GRAVITY_MG        1000
#define S4_FILTER_DIVISOR    4

static bool s4_config_is_valid(s4_terrain_config_t const * p_config);
static void s4_calibration_reset(s4_terrain_context_t * p_context);
static void s4_calibration_add(s4_terrain_context_t * p_context,
                               s4_terrain_sample_t const * p_sample);
static void s4_calibration_finish(s4_terrain_context_t * p_context);
static void s4_fault_set(s4_terrain_context_t * p_context,
                         s4_terrain_fault_t fault);
static void s4_monitor_sample(s4_terrain_context_t * p_context,
                              s4_terrain_sample_t const * p_sample,
                              uint16_t distance_delta_mm,
                              bool b_distance_valid);
static void s4_state_update(s4_terrain_context_t * p_context,
                            int32_t forward_mg,
                            uint16_t distance_delta_mm,
                            bool b_distance_valid);
static void s4_candidate_add(s4_terrain_context_t * p_context,
                             int32_t forward_mg,
                             uint16_t distance_delta_mm,
                             bool b_distance_valid);
static void s4_hump_complete(s4_terrain_context_t * p_context);
static int32_t s4_absolute(int32_t value);

void
s4_terrain_init (s4_terrain_context_t * p_context,
                 s4_terrain_config_t const * p_config)
{
    if (NULL != p_context)
    {
        *p_context = (s4_terrain_context_t) {0};

        if (s4_config_is_valid(p_config))
        {
            p_context->config = *p_config;
            s4_calibration_reset(p_context);
        }
        else
        {
            s4_fault_set(p_context, S4_TERRAIN_FAULT_CALIBRATION);
        }
    }
}

void
s4_terrain_process (s4_terrain_context_t * p_context,
                    s4_terrain_sample_t const * p_sample,
                    uint16_t distance_delta_mm,
                    bool b_distance_valid)
{
    if ((NULL != p_context) && (NULL != p_sample))
    {
        p_context->result.events = 0u;

        if (S4_TERRAIN_STATE_FAULT != p_context->result.state)
        {
            if (!p_sample->b_valid)
            {
                if (p_context->invalid_count < UINT8_MAX)
                {
                    p_context->invalid_count++;
                }

                if (p_context->invalid_count >=
                    p_context->config.invalid_limit)
                {
                    s4_fault_set(p_context, S4_TERRAIN_FAULT_SENSOR);
                }
            }
            else
            {
                uint32_t elapsed_us = p_sample->capture_time_us -
                                      p_context->previous_time_us;
                uint32_t max_gap_us =
                    (uint32_t) p_context->config.max_sample_gap_ms * 1000u;

                p_context->invalid_count = 0u;

                if (p_context->b_have_previous_time &&
                    (elapsed_us > max_gap_us))
                {
                    s4_fault_set(p_context, S4_TERRAIN_FAULT_STALE);
                }
                else
                {
                    p_context->previous_time_us =
                        p_sample->capture_time_us;
                    p_context->b_have_previous_time = true;

                    if (S4_TERRAIN_STATE_CALIBRATING ==
                        p_context->result.state)
                    {
                        s4_calibration_add(p_context, p_sample);
                    }
                    else
                    {
                        s4_monitor_sample(p_context, p_sample,
                                          distance_delta_mm,
                                          b_distance_valid);
                    }
                }
            }
        }
    }
}

void
s4_terrain_request_recovery (s4_terrain_context_t * p_context)
{
    if (NULL != p_context)
    {
        uint16_t maximum_height_mm = p_context->result.maximum_height_mm;
        bool b_maximum_height_valid =
            p_context->result.b_maximum_height_valid;

        s4_calibration_reset(p_context);
        p_context->result.maximum_height_mm = maximum_height_mm;
        p_context->result.b_maximum_height_valid =
            b_maximum_height_valid;
    }
}

void
s4_terrain_start_new_run (s4_terrain_context_t * p_context)
{
    if (NULL != p_context)
    {
        p_context->result.current_height_mm = 0u;
        p_context->result.maximum_height_mm = 0u;
        p_context->result.b_current_height_valid = false;
        p_context->result.b_maximum_height_valid = false;
        p_context->candidate_height_mm = 0u;
        p_context->b_candidate_height_valid = false;
    }
}

s4_terrain_result_t const *
s4_terrain_get_result (s4_terrain_context_t const * p_context)
{
    s4_terrain_result_t const * p_result = NULL;

    if (NULL != p_context)
    {
        p_result = &p_context->result;
    }

    return (p_result);
}

static bool
s4_config_is_valid (s4_terrain_config_t const * p_config)
{
    bool b_valid = false;

    if (NULL != p_config)
    {
        b_valid = (p_config->calibration_samples > 0u) &&
                  (p_config->confirmation_samples > 0u) &&
                  (p_config->invalid_limit > 0u) &&
                  (p_config->max_sample_gap_ms > 0u) &&
                  (p_config->level_threshold_mg <
                   p_config->ascent_threshold_mg);
    }

    return (b_valid);
}

static void
s4_calibration_reset (s4_terrain_context_t * p_context)
{
    s4_terrain_config_t config = p_context->config;
    uint16_t maximum_height_mm = p_context->result.maximum_height_mm;
    bool b_maximum_height_valid =
        p_context->result.b_maximum_height_valid;

    *p_context = (s4_terrain_context_t) {0};
    p_context->config = config;
    p_context->result.state = S4_TERRAIN_STATE_CALIBRATING;
    p_context->result.maximum_height_mm = maximum_height_mm;
    p_context->result.b_maximum_height_valid = b_maximum_height_valid;
    p_context->calibration_min_x = INT16_MAX;
    p_context->calibration_min_y = INT16_MAX;
    p_context->calibration_min_z = INT16_MAX;
    p_context->calibration_max_x = INT16_MIN;
    p_context->calibration_max_y = INT16_MIN;
    p_context->calibration_max_z = INT16_MIN;
}

static void
s4_calibration_add (s4_terrain_context_t * p_context,
                    s4_terrain_sample_t const * p_sample)
{
    p_context->calibration_sum_x += p_sample->acceleration_x_mg;
    p_context->calibration_sum_y += p_sample->acceleration_y_mg;
    p_context->calibration_sum_z += p_sample->acceleration_z_mg;
    p_context->calibration_count++;

    if (p_sample->acceleration_x_mg < p_context->calibration_min_x)
    {
        p_context->calibration_min_x = p_sample->acceleration_x_mg;
    }
    if (p_sample->acceleration_x_mg > p_context->calibration_max_x)
    {
        p_context->calibration_max_x = p_sample->acceleration_x_mg;
    }
    if (p_sample->acceleration_y_mg < p_context->calibration_min_y)
    {
        p_context->calibration_min_y = p_sample->acceleration_y_mg;
    }
    if (p_sample->acceleration_y_mg > p_context->calibration_max_y)
    {
        p_context->calibration_max_y = p_sample->acceleration_y_mg;
    }
    if (p_sample->acceleration_z_mg < p_context->calibration_min_z)
    {
        p_context->calibration_min_z = p_sample->acceleration_z_mg;
    }
    if (p_sample->acceleration_z_mg > p_context->calibration_max_z)
    {
        p_context->calibration_max_z = p_sample->acceleration_z_mg;
    }

    if (p_context->calibration_count >=
        p_context->config.calibration_samples)
    {
        s4_calibration_finish(p_context);
    }
}

static void
s4_calibration_finish (s4_terrain_context_t * p_context)
{
    int32_t span_x = (int32_t) p_context->calibration_max_x -
                     p_context->calibration_min_x;
    int32_t span_y = (int32_t) p_context->calibration_max_y -
                     p_context->calibration_min_y;
    int32_t span_z = (int32_t) p_context->calibration_max_z -
                     p_context->calibration_min_z;
    int32_t allowed_span = p_context->config.calibration_span_mg;

    if ((span_x > allowed_span) || (span_y > allowed_span) ||
        (span_z > allowed_span))
    {
        s4_fault_set(p_context, S4_TERRAIN_FAULT_CALIBRATION);
    }
    else
    {
        int64_t sample_count = p_context->calibration_count;

        /* Each mean remains within the range of the int16_t samples. */
        p_context->offset_x_mg = (int16_t)
            (p_context->calibration_sum_x / sample_count);
        /* Each mean remains within the range of the int16_t samples. */
        p_context->offset_y_mg = (int16_t)
            (p_context->calibration_sum_y / sample_count);
        /* The vertical offset removes the stationary one-g baseline. */
        p_context->offset_z_mg = (int16_t)
            ((p_context->calibration_sum_z / sample_count) -
             S4_GRAVITY_MG);
        p_context->result.state = S4_TERRAIN_STATE_MONITORING;
        p_context->result.fault = S4_TERRAIN_FAULT_NONE;
        p_context->result.events = S4_TERRAIN_EVENT_READY;
    }
}

static void
s4_fault_set (s4_terrain_context_t * p_context,
              s4_terrain_fault_t fault)
{
    p_context->result.state = S4_TERRAIN_STATE_FAULT;
    p_context->result.fault = fault;
    p_context->result.events = S4_TERRAIN_EVENT_FAULT;
    p_context->result.current_height_mm = 0u;
    p_context->result.b_current_height_valid = false;
    p_context->b_candidate_height_valid = false;
}

static void
s4_monitor_sample (s4_terrain_context_t * p_context,
                   s4_terrain_sample_t const * p_sample,
                   uint16_t distance_delta_mm,
                   bool b_distance_valid)
{
    int32_t forward_mg = (int32_t) p_sample->acceleration_x_mg -
                         p_context->offset_x_mg;
    int32_t lateral_mg = (int32_t) p_sample->acceleration_y_mg -
                         p_context->offset_y_mg;
    int32_t vertical_mg = (int32_t) p_sample->acceleration_z_mg -
                          p_context->offset_z_mg - S4_GRAVITY_MG;
    int32_t dynamic_mg = s4_absolute(forward_mg) +
                         s4_absolute(lateral_mg) +
                         s4_absolute(vertical_mg);

    p_context->filtered_x_mg +=
        (forward_mg - p_context->filtered_x_mg) / S4_FILTER_DIVISOR;

    /* The filter input and accumulator are bounded by int16_t samples. */
    p_context->result.forward_tilt_mg =
        (int16_t) p_context->filtered_x_mg;

    if (dynamic_mg >= p_context->config.impact_threshold_mg)
    {
        if (!p_context->b_impact_active)
        {
            p_context->result.events |= S4_TERRAIN_EVENT_IMPACT;
            p_context->b_impact_active = true;
        }
    }
    else
    {
        p_context->b_impact_active = false;
    }

    s4_state_update(p_context, forward_mg, distance_delta_mm,
                    b_distance_valid);
}

static void
s4_state_update (s4_terrain_context_t * p_context,
                 int32_t forward_mg,
                 uint16_t distance_delta_mm,
                 bool b_distance_valid)
{
    int32_t ascent_threshold = p_context->config.ascent_threshold_mg;
    int32_t level_threshold = p_context->config.level_threshold_mg;

    switch (p_context->result.state)
    {
      case S4_TERRAIN_STATE_MONITORING:
        if (forward_mg >= ascent_threshold)
        {
            p_context->confirmation_count++;
            if (p_context->confirmation_count >=
                p_context->config.confirmation_samples)
            {
                p_context->result.state = S4_TERRAIN_STATE_ASCENDING;
                p_context->candidate_height_mm = 0u;
                p_context->b_candidate_height_valid = true;
                p_context->confirmation_count = 0u;
            }
        }
        else
        {
            p_context->confirmation_count = 0u;
        }
      break;

      case S4_TERRAIN_STATE_ASCENDING:
        s4_candidate_add(p_context, forward_mg, distance_delta_mm,
                         b_distance_valid);
        if (forward_mg <= -ascent_threshold)
        {
            p_context->confirmation_count++;
            if (p_context->confirmation_count >=
                p_context->config.confirmation_samples)
            {
                p_context->result.state = S4_TERRAIN_STATE_DESCENDING;
                p_context->confirmation_count = 0u;
            }
        }
        else
        {
            p_context->confirmation_count = 0u;
        }
      break;

      case S4_TERRAIN_STATE_DESCENDING:
        if (s4_absolute(forward_mg) <= level_threshold)
        {
            p_context->confirmation_count++;
            if (p_context->confirmation_count >=
                p_context->config.confirmation_samples)
            {
                s4_hump_complete(p_context);
            }
        }
        else
        {
            p_context->confirmation_count = 0u;
        }
      break;

      default:
        s4_fault_set(p_context, S4_TERRAIN_FAULT_SENSOR);
      break;
    }
}

static void
s4_candidate_add (s4_terrain_context_t * p_context,
                  int32_t forward_mg,
                  uint16_t distance_delta_mm,
                  bool b_distance_valid)
{
    if ((!b_distance_valid) || (forward_mg < 0))
    {
        if (!b_distance_valid)
        {
            p_context->b_candidate_height_valid = false;
        }
    }
    else
    {
        uint32_t height_delta_mm =
            ((uint32_t) distance_delta_mm * (uint32_t) forward_mg) /
            S4_GRAVITY_MG;
        uint32_t candidate = p_context->candidate_height_mm +
                             height_delta_mm;

        if (candidate > UINT16_MAX)
        {
            p_context->candidate_height_mm = UINT16_MAX;
            p_context->b_candidate_height_valid = false;
        }
        else
        {
            /* The preceding bound proves the result fits uint16_t. */
            p_context->candidate_height_mm = (uint16_t) candidate;
        }
    }
}

static void
s4_hump_complete (s4_terrain_context_t * p_context)
{
    p_context->result.current_height_mm =
        p_context->candidate_height_mm;
    p_context->result.b_current_height_valid =
        p_context->b_candidate_height_valid;

    if (p_context->b_candidate_height_valid &&
        ((!p_context->result.b_maximum_height_valid) ||
         (p_context->candidate_height_mm >
          p_context->result.maximum_height_mm)))
    {
        p_context->result.maximum_height_mm =
            p_context->candidate_height_mm;
        p_context->result.b_maximum_height_valid = true;
    }

    p_context->result.events |= S4_TERRAIN_EVENT_HUMP_COMPLETE;
    p_context->result.state = S4_TERRAIN_STATE_MONITORING;
    p_context->confirmation_count = 0u;
    p_context->candidate_height_mm = 0u;
    p_context->b_candidate_height_valid = false;
}

static int32_t
s4_absolute (int32_t value)
{
    int32_t absolute = value;

    if (value < 0)
    {
        absolute = -value;
    }

    return (absolute);
}

/* End of file. */
