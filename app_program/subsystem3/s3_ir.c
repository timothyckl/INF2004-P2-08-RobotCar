/** @file s3_ir.c Pure logic; no GPIO or motor access. */
#include "s3_ir.h"
#include <stddef.h>

#define S3_IR_FILTER_DIVISOR         2u
#define S3_IR_ADAPTATION_SAMPLES     8u

static uint16_t s3_ir_threshold(uint16_t baseline, uint16_t margin);

bool s3_ir_filter(s3_filter_t * p_filter, uint8_t mask, uint8_t required)
{
    bool settled = false;

    if ((NULL != p_filter) && (mask < S3_IR_PATTERNS) && (required > 0U))
    {
        if ((0U == p_filter->repetitions) ||
            (mask != p_filter->candidate))
        {
            p_filter->candidate = mask;
            p_filter->repetitions = 1U;
        }
        else if (p_filter->repetitions < required)
        {
            ++p_filter->repetitions;
        }
        else
        {
            /* Saturation prevents counter wrap on a steady surface. */
        }

        settled = (p_filter->repetitions >= required);
        if (settled)
        {
            p_filter->stable = mask;
        }
        p_filter->ready = settled;
    }
    else if (NULL != p_filter)
    {
        p_filter->ready = false;
        p_filter->repetitions = 0U;
    }
    else
    {
        /* An absent filter cannot produce a settled sample. */
    }

    return settled;
}

bool
s3_ir_adaptive_init (s3_ir_adaptive_t * p_state,
                     s3_ir_adaptive_config_t const * p_config)
{
    bool b_valid = false;

    if ((NULL != p_state) && (NULL != p_config))
    {
        b_valid = (p_config->minimum_white <= p_config->initial_white)
            && (p_config->initial_white <= p_config->maximum_white)
            && (p_config->maximum_white <= S3_IR_ADC_MAX)
            && (p_config->white_margin < p_config->black_margin)
            && (p_config->black_margin > 0u)
            && (p_config->maximum_white <=
                (S3_IR_ADC_MAX - p_config->black_margin));

        p_state->config = *p_config;
        p_state->white_baseline = p_config->initial_white;
        p_state->filtered = p_config->initial_white;
        p_state->threshold = s3_ir_threshold(p_config->initial_white,
                                             p_config->black_margin);
        p_state->adaptation_count = 0u;
        p_state->b_black = false;
        p_state->b_valid = b_valid;
    }

    return (b_valid);
}

bool
s3_ir_adaptive_update (s3_ir_adaptive_t * p_state, uint16_t sample)
{
    uint16_t release_threshold;

    if ((NULL == p_state) || (!p_state->b_valid)
        || (sample > S3_IR_ADC_MAX))
    {
        return (false);
    }

    if (sample > p_state->filtered)
    {
        uint16_t const difference = sample - p_state->filtered;

        p_state->filtered += (uint16_t)((difference + 1u)
                                       / S3_IR_FILTER_DIVISOR);
    }
    else
    {
        uint16_t const difference = p_state->filtered - sample;

        p_state->filtered -= (uint16_t)((difference + 1u)
                                       / S3_IR_FILTER_DIVISOR);
    }

    p_state->threshold = s3_ir_threshold(p_state->white_baseline,
                                         p_state->config.black_margin);
    release_threshold = s3_ir_threshold(p_state->white_baseline,
                                        p_state->config.white_margin);

    if (p_state->b_black)
    {
        if (p_state->filtered <= release_threshold)
        {
            p_state->b_black = false;
        }
    }
    else if (p_state->filtered >= p_state->threshold)
    {
        p_state->b_black = true;
    }
    else
    {
        /* The reading remains on the white side of the threshold. */
    }

    if (!p_state->b_black)
    {
        p_state->adaptation_count++;
        if (p_state->adaptation_count >= S3_IR_ADAPTATION_SAMPLES)
        {
            if ((p_state->filtered > p_state->white_baseline)
                && (p_state->white_baseline
                    < p_state->config.maximum_white))
            {
                p_state->white_baseline++;
            }
            else if ((p_state->filtered < p_state->white_baseline)
                     && (p_state->white_baseline
                         > p_state->config.minimum_white))
            {
                p_state->white_baseline--;
            }
            else
            {
                /* The baseline is settled or held at its safe limit. */
            }
            p_state->adaptation_count = 0u;
            p_state->threshold = s3_ir_threshold(
                p_state->white_baseline, p_state->config.black_margin);
        }
    }
    else
    {
        p_state->adaptation_count = 0u;
    }

    return (p_state->b_black);
}

uint8_t
s3_ir_adaptive_strength (s3_ir_adaptive_t const * p_state,
                         uint16_t black_reference)
{
    uint8_t strength = 0u;

    if ((NULL != p_state) && p_state->b_valid
        && (black_reference > p_state->white_baseline)
        && (p_state->filtered > p_state->white_baseline))
    {
        uint16_t const difference = p_state->filtered
                                  - p_state->white_baseline;
        uint16_t const span = black_reference - p_state->white_baseline;

        if (difference >= span)
        {
            strength = UINT8_C(100);
        }
        else
        {
            uint32_t const scaled = ((uint32_t)difference * UINT32_C(100))
                                  + ((uint32_t)span / UINT32_C(2));

            strength = (uint8_t)(scaled / (uint32_t)span);
        }
    }

    return (strength);
}

static uint16_t
s3_ir_threshold (uint16_t baseline, uint16_t margin)
{
    uint16_t threshold = S3_IR_ADC_MAX;

    if (baseline <= (S3_IR_ADC_MAX - margin))
    {
        threshold = baseline + margin;
    }

    return (threshold);
}

s3_line_t s3_ir_line(uint8_t mask,
                     s3_line_t const decisions[S3_IR_PATTERNS])
{
    s3_line_t result = S3_LINE_UNKNOWN;

    if ((NULL != decisions) && (mask < S3_IR_PATTERNS))
    {
        result = decisions[mask];
    }

    return result;
}

s3_line_t
s3_ir_follow_decision (uint8_t black_mask)
{
    static s3_line_t const decisions[S3_IR_PATTERNS] =
    {
        S3_LINE_STRAIGHT,
        S3_LINE_LEFT,
        S3_LINE_RIGHT,
        S3_LINE_JUNCTION
    };

    return (s3_ir_line(black_mask, decisions));
}

/** End of file. */

