/**
 * @file terrain_logic.c
 * @brief Deterministic IMU calibration, hump and impact detection.
 */
#include "terrain_logic.h"

#include <limits.h>
#include <stddef.h>

#define S4_GRAVITY_MG        1000
#define S4_UM_PER_MM         1000

static bool s4_config_is_valid(s4_terrain_config_t const * p_config);
static void s4_calibration_reset(s4_terrain_context_t * p_context);
static void s4_calibration_add(s4_terrain_context_t * p_context,
                               s4_terrain_sample_t const * p_sample);
static void s4_calibration_finish(s4_terrain_context_t * p_context);
static void s4_fault_set(s4_terrain_context_t * p_context,
                         s4_terrain_fault_t fault);
static void s4_monitor_sample(s4_terrain_context_t * p_context,
                              s4_terrain_sample_t const * p_sample,
                              bool b_pitch_valid);
static void s4_average_add(s4_terrain_context_t * p_context,
                           int32_t forward_mg,
                           int32_t lateral_mg);
static int32_t s4_hump_height_um(s4_terrain_context_t const * p_context,
                                 bool b_pitch_valid,
                                 s4_hump_contact_t * p_contact);
static void s4_hump_update(s4_terrain_context_t * p_context,
                           uint32_t capture_time_us,
                           int32_t height_um,
                           s4_hump_contact_t contact);
static void s4_hump_complete(s4_terrain_context_t * p_context);
static void s4_hump_clear(s4_terrain_context_t * p_context);
static void s4_lift_update(s4_terrain_context_t * p_context,
                           int32_t forward_mg,
                           int32_t lateral_mg,
                           int32_t vertical_mg);
static uint8_t s4_count_at(uint8_t count, int32_t value, int32_t limit);
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
                    bool b_pitch_valid)
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
                /* Widening: max_sample_gap_ms is uint16_t, so x 1000 is at
                   most 65,535,000 and fits uint32_t. */
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
                                          b_pitch_valid);
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
        p_context->result.hump_count = 0u;
        p_context->result.hump_contact = S4_HUMP_CONTACT_NONE;
    }
}

/*
 * Hump and impact detection run only while enabled (decisions 12 and 14):
 * the caller disables them while the robot is not driving, so lifting the
 * car, a GP20 press and braking at a stop are neither humps nor impacts.
 * Disabling drops any hump in progress without reporting it; the tilt
 * average and the impact edge keep being tracked, so re-enabling during a
 * shake does not report a stale impact.
 */
void
s4_terrain_set_detect_enable (s4_terrain_context_t * p_context,
                            bool b_enable)
{
    if (NULL != p_context)
    {
        p_context->b_detect_disabled = !b_enable;
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
                  (p_config->max_hump_ms > 0u) &&
                  (p_config->track_mm > 0u) &&
                  (p_config->wheelbase_mm > 0u) &&
                  (p_config->level_um < p_config->rise_um) &&
                  (p_config->average_samples > 0u) &&
                  (p_config->average_samples <= S4_AVERAGE_MAX_SAMPLES) &&
                  (p_config->height_scale_permille > 0u) &&
                  (p_config->height_scale_permille <=
                   S4_HEIGHT_SCALE_MAX_PERMILLE) &&
                  (p_config->lift_up_mg > 0u) &&
                  (p_config->lift_tilt_mg > 0u) &&
                  (p_config->lift_samples > 0u) &&
                  (p_config->lift_raw_tilt_mg > 0u) &&
                  (p_config->lift_raw_samples > 0u);
    }

    return (b_valid);
}

static void
s4_calibration_reset (s4_terrain_context_t * p_context)
{
    s4_terrain_config_t config = p_context->config;
    s4_terrain_result_t const kept = p_context->result;

    /* The run's hump results survive recalibration (a pick-up recovery);
     * only s4_terrain_start_new_run() clears them. */
    *p_context = (s4_terrain_context_t) {0};
    p_context->config = config;
    p_context->result.state = S4_TERRAIN_STATE_CALIBRATING;
    p_context->result.current_height_mm = kept.current_height_mm;
    p_context->result.b_current_height_valid = kept.b_current_height_valid;
    p_context->result.maximum_height_mm = kept.maximum_height_mm;
    p_context->result.b_maximum_height_valid = kept.b_maximum_height_valid;
    p_context->result.hump_count = kept.hump_count;
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
    /* Widening: the limits are int16_t, so each difference is within
       +/-65535 and fits int32_t. */
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
    s4_hump_clear(p_context);
}

static void
s4_monitor_sample (s4_terrain_context_t * p_context,
                   s4_terrain_sample_t const * p_sample,
                   bool b_pitch_valid)
{
    /* Widening: samples and offsets are int16_t, so each difference (and
       minus 1000 mg of gravity) fits int32_t. */
    int32_t forward_mg = (int32_t) p_sample->acceleration_x_mg -
                         p_context->offset_x_mg;
    int32_t lateral_mg = (int32_t) p_sample->acceleration_y_mg -
                         p_context->offset_y_mg;
    int32_t vertical_mg = (int32_t) p_sample->acceleration_z_mg -
                          p_context->offset_z_mg - S4_GRAVITY_MG;
    int32_t dynamic_mg = s4_absolute(forward_mg) +
                         s4_absolute(lateral_mg) +
                         s4_absolute(vertical_mg);
    s4_hump_contact_t contact = S4_HUMP_CONTACT_NONE;
    int32_t height_um;

    s4_average_add(p_context, forward_mg, lateral_mg);

    /* Each average lies within the range of the int16_t samples. */
    p_context->result.forward_tilt_mg = (int16_t) p_context->filtered_x_mg;
    p_context->result.lateral_tilt_mg = (int16_t) p_context->filtered_y_mg;

    if (dynamic_mg >= p_context->config.impact_threshold_mg)
    {
        if ((!p_context->b_impact_active) && (!p_context->b_detect_disabled))
        {
            p_context->result.events |= S4_TERRAIN_EVENT_IMPACT;
        }
        p_context->b_impact_active = true;
    }
    else
    {
        p_context->b_impact_active = false;
    }

    s4_lift_update(p_context, forward_mg, lateral_mg, vertical_mg);

    height_um = s4_hump_height_um(p_context, b_pitch_valid, &contact);
    s4_hump_update(p_context, p_sample->capture_time_us, height_um,
                   contact);
}

/*
 * Moving average of the last average_samples tilts (decision 10): driving
 * vibration averages out, while a hump, crossed in seconds at line-follow
 * speed, keeps its height. Until the window fills, the mean of the samples
 * held so far is used. Inputs are differences of int16_t values, so the sum
 * of at most S4_AVERAGE_MAX_SAMPLES of them fits in int32_t.
 */
static void
s4_average_add (s4_terrain_context_t * p_context,
                int32_t forward_mg,
                int32_t lateral_mg)
{
    uint8_t const length = p_context->config.average_samples;
    uint8_t const index = p_context->average_index;

    if (p_context->average_count >= length)
    {
        /* The window is full: the oldest sample leaves the sum. */
        p_context->average_sum_x_mg -= p_context->average_x_mg[index];
        p_context->average_sum_y_mg -= p_context->average_y_mg[index];
    }
    else
    {
        p_context->average_count++;
    }

    p_context->average_x_mg[index] = forward_mg;
    p_context->average_y_mg[index] = lateral_mg;
    p_context->average_sum_x_mg += forward_mg;
    p_context->average_sum_y_mg += lateral_mg;
    /* index < length <= S4_AVERAGE_MAX_SAMPLES (16), so index + 1 fits. */
    p_context->average_index =
        ((uint8_t) (index + 1u) >= length) ? 0u : (uint8_t) (index + 1u);

    p_context->filtered_x_mg =
        p_context->average_sum_x_mg / (int32_t) p_context->average_count;
    p_context->filtered_y_mg =
        p_context->average_sum_y_mg / (int32_t) p_context->average_count;
}

/*
 * Contact heights (micrometres, mg x mm) of the plane through the three
 * contacts, relative to the wheel axle's midpoint: left wheel +roll x
 * track/2, right wheel -roll x track/2, caster +pitch x wheelbase. With
 * the lowest contact on the floor, the highest one's rise is
 * highest - lowest. When forward acceleration makes pitch unreadable
 * (b_pitch_valid false) the caster is taken to be level with the lower
 * wheel, which leaves the roll-only estimate of a one-wheel hump.
 * Products are bounded: |tilt| < 4096 mg, distances < 65536 mm.
 */
static int32_t
s4_hump_height_um (s4_terrain_context_t const * p_context,
                   bool b_pitch_valid,
                   s4_hump_contact_t * p_contact)
{
    int32_t left_um = (p_context->filtered_y_mg *
                       (int32_t) p_context->config.track_mm) / 2;
    int32_t right_um = -left_um;
    int32_t lower_wheel_um = (left_um < right_um) ? left_um : right_um;
    int32_t upper_wheel_um = (left_um < right_um) ? right_um : left_um;
    int32_t caster_um = lower_wheel_um;
    int32_t highest_um;
    int32_t lowest_um;

    if (b_pitch_valid)
    {
        /* filtered_x_mg is an average of +/-2 g samples (|x| <= 2000 mg) and
           wheelbase_mm is uint16_t, so the product is below 1.4 x 10^8 and
           fits int32_t. */
        caster_um = p_context->filtered_x_mg *
                    (int32_t) p_context->config.wheelbase_mm;
    }

    highest_um = (caster_um > upper_wheel_um) ? caster_um : upper_wheel_um;
    lowest_um = (caster_um < lower_wheel_um) ? caster_um : lower_wheel_um;

    if (caster_um > upper_wheel_um)
    {
        *p_contact = S4_HUMP_CONTACT_CASTER;
    }
    else if ((2 * (upper_wheel_um - lower_wheel_um)) <
             (upper_wheel_um - caster_um))
    {
        /* Wheels nearly level with each other and well above the
         * caster: both wheels are raised (one wheel alone puts the
         * caster level with the lower wheel). */
        *p_contact = S4_HUMP_CONTACT_BOTH_WHEELS;
    }
    else if (left_um >= right_um)
    {
        *p_contact = S4_HUMP_CONTACT_LEFT;
    }
    else
    {
        *p_contact = S4_HUMP_CONTACT_RIGHT;
    }

    /* Scaled to the measured hump (decision 12). The difference is below
     * 2 x 4096 mg x 65535 mm (about 5.4e8 um), the product fits in
     * int64_t, and with the scale at most 2000 per mille the quotient is
     * below 1.1e9, inside int32_t. */
    return ((int32_t) (((int64_t) (highest_um - lowest_um) *
                        (int64_t) p_context->config.height_scale_permille) /
                       1000));
}

static void
s4_hump_update (s4_terrain_context_t * p_context,
                uint32_t capture_time_us,
                int32_t height_um,
                s4_hump_contact_t contact)
{
    /* Widening: max_hump_ms is uint16_t, so x 1000 fits uint32_t. */
    uint32_t max_hump_us =
        (uint32_t) p_context->config.max_hump_ms * 1000u;

    if (p_context->b_detect_disabled)
    {
        s4_hump_clear(p_context);
    }
    else if (p_context->b_wait_level)
    {
        /* After a hump was dropped, re-arm only once level again. */
        if (height_um <= (int32_t) p_context->config.level_um)
        {
            p_context->b_wait_level = false;
        }
    }
    else if (S4_TERRAIN_STATE_MONITORING == p_context->result.state)
    {
        if (height_um >= (int32_t) p_context->config.rise_um)
        {
            if (0u == p_context->confirmation_count)
            {
                p_context->hump_start_us = capture_time_us;
                p_context->peak_height_um = 0;
            }
            if (height_um > p_context->peak_height_um)
            {
                p_context->peak_height_um = height_um;
                p_context->peak_contact = contact;
            }
            p_context->confirmation_count++;
            if (p_context->confirmation_count >=
                p_context->config.confirmation_samples)
            {
                p_context->result.state = S4_TERRAIN_STATE_ON_HUMP;
                p_context->confirmation_count = 0u;
            }
        }
        else
        {
            p_context->confirmation_count = 0u;
        }
    }
    else if ((capture_time_us - p_context->hump_start_us) > max_hump_us)
    {
        /* Too long to be a hump (for example the car stopped with a
         * wheel raised); unsigned subtraction is wrap-safe. */
        s4_hump_clear(p_context);
        p_context->b_wait_level = true;
    }
    else
    {
        if (height_um > p_context->peak_height_um)
        {
            p_context->peak_height_um = height_um;
            p_context->peak_contact = contact;
        }

        if (height_um <= (int32_t) p_context->config.level_um)
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
    }
}

static void
s4_hump_complete (s4_terrain_context_t * p_context)
{
    /* Rounded to the nearest mm; the peak is a positive height below
     * 2 x 4096 mg x 65535 mm, well inside int32_t, and below 65536 mm
     * for any real chassis. */
    uint16_t height_mm = (uint16_t)
        ((p_context->peak_height_um + (S4_UM_PER_MM / 2)) / S4_UM_PER_MM);

    p_context->result.current_height_mm = height_mm;
    p_context->result.b_current_height_valid = true;
    p_context->result.hump_contact = p_context->peak_contact;

    if (p_context->result.hump_count < UINT8_MAX)
    {
        p_context->result.hump_count++;
    }

    if ((!p_context->result.b_maximum_height_valid) ||
        (height_mm > p_context->result.maximum_height_mm))
    {
        p_context->result.maximum_height_mm = height_mm;
        p_context->result.b_maximum_height_valid = true;
    }

    p_context->result.events |= S4_TERRAIN_EVENT_HUMP_COMPLETE;
    s4_hump_clear(p_context);
}

static void
s4_hump_clear (s4_terrain_context_t * p_context)
{
    if (S4_TERRAIN_STATE_ON_HUMP == p_context->result.state)
    {
        p_context->result.state = S4_TERRAIN_STATE_MONITORING;
    }
    p_context->confirmation_count = 0u;
    p_context->peak_height_um = 0;
    p_context->peak_contact = S4_HUMP_CONTACT_NONE;
}

/*
 * Lift detection (decision 15). A hand lifting the car accelerates it
 * upwards harder than driving does, and tips it for longer than a driving
 * jolt; a car carried at an angle holds a tilt that driving never reaches.
 * Upward only: dropping off a hump reads as less than one g. Disabling
 * re-arms it, so each drive reports at most one lift. The inputs are
 * differences of int16_t values, so they fit.
 */
static void
s4_lift_update (s4_terrain_context_t * p_context,
                int32_t forward_mg,
                int32_t lateral_mg,
                int32_t vertical_mg)
{
    s4_terrain_config_t const * p_config = &p_context->config;
    int32_t const tilt_limit_mg = (int32_t) p_config->lift_tilt_mg;
    int32_t raw_tilt_mg = s4_absolute(forward_mg);
    bool b_lifted;

    if (s4_absolute(lateral_mg) > raw_tilt_mg)
    {
        raw_tilt_mg = s4_absolute(lateral_mg);
    }

    if (p_context->b_detect_disabled)
    {
        p_context->lift_count = 0u;
        p_context->lift_tilt_count = 0u;
        p_context->b_lift_reported = false;
    }
    else
    {
        /* Widening: the lift limits are uint16_t, which int32_t holds, so
           they compare with signed readings. */
        p_context->lift_count =
            s4_count_at(p_context->lift_count, vertical_mg,
                        (int32_t) p_config->lift_up_mg);
        p_context->lift_tilt_count =
            s4_count_at(p_context->lift_tilt_count, raw_tilt_mg,
                        (int32_t) p_config->lift_raw_tilt_mg);

        b_lifted = (p_context->lift_count >= p_config->lift_samples) ||
                   (p_context->lift_tilt_count >=
                    p_config->lift_raw_samples) ||
                   (s4_absolute(p_context->filtered_x_mg) >= tilt_limit_mg) ||
                   (s4_absolute(p_context->filtered_y_mg) >= tilt_limit_mg);

        if (b_lifted && (!p_context->b_lift_reported))
        {
            p_context->result.events |= S4_TERRAIN_EVENT_LIFT;
            p_context->b_lift_reported = true;
        }
    }
}

/* Consecutive samples with value at or above limit, saturating. */
static uint8_t
s4_count_at (uint8_t count,
             int32_t value,
             int32_t limit)
{
    uint8_t next = 0u;

    if (value >= limit)
    {
        next = count;
        if (next < UINT8_MAX)
        {
            next++;
        }
    }

    return (next);
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
