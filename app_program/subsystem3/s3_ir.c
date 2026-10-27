/** @file s3_ir.c Pure logic; no GPIO or motor access. */
#include "s3_ir.h"
#include <stddef.h>

bool s3_ir_calibrate(s3_calibration_t * p_cal, uint8_t white_levels,
                     uint8_t black_levels)
{
    bool valid = false;

    if (NULL != p_cal)
    {
        valid = (white_levels < S3_IR_PATTERNS) &&
                (black_levels < S3_IR_PATTERNS) &&
                ((white_levels ^ black_levels) == (S3_IR_PATTERNS - 1U));
        p_cal->valid = valid;
        p_cal->black_levels = black_levels;
    }

    return valid;
}

bool s3_ir_normalize(s3_calibration_t const * p_cal, uint8_t levels,
                    uint8_t * p_black_mask)
{
    bool valid = false;

    if ((NULL != p_cal) && (NULL != p_black_mask))
    {
        valid = p_cal->valid && (levels < S3_IR_PATTERNS);
        if (valid)
        {
            /* Only the three sensor bits are retained. */
            *p_black_mask = (uint8_t)((~(levels ^ p_cal->black_levels)) &
                                      (S3_IR_PATTERNS - 1U));
        }
    }

    return valid;
}

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

/** End of file. */

