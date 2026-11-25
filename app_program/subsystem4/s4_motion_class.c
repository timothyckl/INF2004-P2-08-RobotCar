/**
 * @file s4_motion_class.c
 * @brief Motion classification and encoder-estimated yaw rate (pure).
 */
#include "s4_motion_class.h"

#include <stddef.h>

/* 1800 / pi = 572.9578 (0.1 deg per rad), scaled by 10,000. */
#define S4_YAW_DDEG_PER_RAD_X10K    5729578
#define S4_YAW_SCALE                10000

static s4_motion_class_t s4_motion_raw(
    s4_motion_classifier_t * p_class,
    s4_motion_class_config_t const * p_config,
    s4_motion_input_t const * p_input);
static int32_t s4_magnitude(int32_t value);

void
s4_speed_gate_reset (s4_speed_gate_t * p_gate)
{
    if (NULL != p_gate)
    {
        p_gate->b_have_previous = false;
    }
}

bool
s4_speed_gate_step (s4_speed_gate_t * p_gate, int32_t speed_mm_s,
                    int32_t change_mm_s, uint8_t hold_samples)
{
    bool b_steady = false;

    if (NULL == p_gate)
    {
        return (false);
    }

    if (p_gate->b_have_previous)
    {
        /* Both speeds are means of two int16_t values, so the difference
           fits int32_t. */
        int32_t const change = speed_mm_s - p_gate->speed_mm_s;

        if ((change > change_mm_s) || (change < -change_mm_s))
        {
            p_gate->hold = hold_samples;
            p_gate->direction = (change > 0) ? 1 : -1;
        }
        else if (p_gate->hold > 0u)
        {
            p_gate->hold--;
        }
        else
        {
            p_gate->direction = 0;
            b_steady = true;
        }
    }

    p_gate->speed_mm_s = speed_mm_s;
    p_gate->b_have_previous = true;

    return (b_steady);
}

int16_t
s4_motion_yaw_ddeg_s (int16_t left_mm_s, int16_t right_mm_s,
                      uint16_t track_mm)
{
    int64_t yaw = 0;

    if (0u != track_mm)
    {
        /* |right - left| <= 65,535, so the product is below 3.8e11 and
           fits int64_t; the divisor is at most 6.6e8. */
        int64_t const numerator =
            ((int64_t) right_mm_s - (int64_t) left_mm_s)
            * S4_YAW_DDEG_PER_RAD_X10K;
        int64_t const divisor = (int64_t) track_mm * S4_YAW_SCALE;

        /* Round half away from zero. */
        yaw = (numerator >= 0) ? ((numerator + (divisor / 2)) / divisor)
                               : ((numerator - (divisor / 2)) / divisor);
    }

    /* Saturate to int16_t; the cast is then in range. */
    yaw = (yaw > INT16_MAX) ? INT16_MAX : yaw;
    yaw = (yaw < INT16_MIN) ? INT16_MIN : yaw;

    return ((int16_t) yaw);
}

void
s4_motion_class_init (s4_motion_classifier_t * p_class)
{
    if (NULL != p_class)
    {
        s4_motion_classifier_t const zero = {0};

        *p_class = zero;
    }
}

s4_motion_class_t
s4_motion_class_step (s4_motion_classifier_t * p_class,
                      s4_motion_class_config_t const * p_config,
                      s4_motion_input_t const * p_input)
{
    s4_motion_class_t raw;

    if (NULL == p_class)
    {
        return (S4_MOTION_UNKNOWN);
    }
    if ((NULL == p_config) || (NULL == p_input)
        || (0u == p_config->confirm_samples))
    {
        p_class->state = S4_MOTION_UNKNOWN;
        p_class->yaw_ddeg_s = 0;
        return (p_class->state);
    }

    raw = s4_motion_raw(p_class, p_config, p_input);

    if ((S4_MOTION_UNKNOWN == raw) || (S4_MOTION_IMPACT == raw)
        || (raw == p_class->state))
    {
        /* Taken at once, or no change: any other candidate is dropped. */
        p_class->state = raw;
        p_class->candidate = raw;
        p_class->candidate_count = 0u;
    }
    else
    {
        if (raw == p_class->candidate)
        {
            /* Bounded by confirm_samples (uint8_t) below. */
            p_class->candidate_count++;
        }
        else
        {
            p_class->candidate = raw;
            p_class->candidate_count = 1u;
        }
        if (p_class->candidate_count >= p_config->confirm_samples)
        {
            p_class->state = raw;
            p_class->candidate_count = 0u;
        }
    }

    return (p_class->state);
}

char const *
s4_motion_class_name (s4_motion_class_t state)
{
    static char const * const p_names[S4_MOTION_CLASS_COUNT + 1u] =
    {
        "unknown", "stationary", "cruising", "accelerating", "braking",
        "turning", "climbing", "descending", "impact", "?"
    };

    return ((state < S4_MOTION_CLASS_COUNT) ? p_names[state]
                                            : p_names[S4_MOTION_CLASS_COUNT]);
}

/* The undebounced state of this sample, by the priority in the header.
   Also updates the yaw rate and the impact hold. */
static s4_motion_class_t
s4_motion_raw (s4_motion_classifier_t * p_class,
               s4_motion_class_config_t const * p_config,
               s4_motion_input_t const * p_input)
{
    s4_motion_class_t raw = S4_MOTION_CRUISING;
    bool const b_fresh = p_input->b_motion_valid
                         && (p_input->age_ms <= p_config->max_age_ms);

    if (p_input->b_terrain_valid && p_input->b_impact)
    {
        p_class->impact_hold = p_config->confirm_samples;
    }

    if (!b_fresh)
    {
        p_class->yaw_ddeg_s = 0;
        p_class->impact_hold = 0u;
        return (S4_MOTION_UNKNOWN);
    }

    p_class->yaw_ddeg_s = s4_motion_yaw_ddeg_s(p_input->left_mm_s,
                                               p_input->right_mm_s,
                                               p_config->track_mm);

    if (p_class->impact_hold > 0u)
    {
        p_class->impact_hold--;
        raw = S4_MOTION_IMPACT;
    }
    else if (p_input->b_terrain_valid && p_input->b_on_hump)
    {
        /* Widening: the band is at most 4.3e9 um, so the comparison is
           made in int64_t. */
        raw = (((int64_t) p_input->height_um
                + (int64_t) p_config->hump_band_um)
               >= (int64_t) p_input->peak_um)
              ? S4_MOTION_CLIMBING : S4_MOTION_DESCENDING;
    }
    else if (s4_magnitude(p_class->yaw_ddeg_s)
             >= (int32_t) p_config->turn_ddeg_s)
    {
        raw = S4_MOTION_TURNING;
    }
    else if ((s4_magnitude(p_input->left_mm_s)
              <= (int32_t) p_config->still_mm_s)
             && (s4_magnitude(p_input->right_mm_s)
                 <= (int32_t) p_config->still_mm_s))
    {
        raw = S4_MOTION_STATIONARY;
    }
    else if (p_input->speed_change > 0)
    {
        raw = S4_MOTION_ACCELERATING;
    }
    else if (p_input->speed_change < 0)
    {
        raw = S4_MOTION_BRAKING;
    }
    else
    {
        /* Moving steadily in a straight line. */
    }

    return (raw);
}

static int32_t
s4_magnitude (int32_t value)
{
    /* Inputs are int16_t values widened, so negation cannot overflow. */
    return ((value < 0) ? -value : value);
}

/* End of file. */
