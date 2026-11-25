/** @file s3_bar.c Capture is independent of GPIO and RTOS scheduling. */
#include "s3_bar.h"
#include <stddef.h>

#define S3_BAR_SYMBOL_ERROR_MAX    1U

void s3_bar_begin(s3_capture_t * p_capture, uint32_t now_us)
{
    if (NULL != p_capture)
    {
        p_capture->last_us = now_us;
        p_capture->count = 0U;
        p_capture->b_black = true;
        p_capture->b_active = true;
        p_capture->b_failed = false;
    }
}

bool s3_bar_expire(s3_capture_t * p_capture, uint32_t now_us,
                   uint32_t timeout_us)
{
    bool b_expired = false;

    if ((NULL != p_capture) && p_capture->b_active)
    {
        b_expired = (0U == timeout_us) ||
                  ((now_us - p_capture->last_us) > timeout_us);
        if (b_expired)
        {
            p_capture->b_active = false;
            p_capture->b_failed = true;
        }
    }

    return b_expired;
}

bool s3_bar_edge(s3_capture_t * p_capture, bool b_black, uint32_t now_us,
                 uint32_t timeout_us)
{
    bool b_accepted = false;

    if ((NULL != p_capture) && p_capture->b_active)
    {
        bool b_expired = s3_bar_expire(p_capture, now_us, timeout_us);

        if ((!b_expired) && (b_black != p_capture->b_black))
        {
            uint32_t duration = now_us - p_capture->last_us;

            if ((0U == duration) || (p_capture->count >= S3_BAR_CAPACITY))
            {
                p_capture->b_failed = true;
                p_capture->b_active = false;
            }
            else
            {
                p_capture->runs[p_capture->count] = duration;
                ++p_capture->count;
                p_capture->last_us = now_us;
                p_capture->b_black = b_black;
                b_accepted = true;
            }
        }
    }

    return b_accepted;
}

bool s3_bar_finish(s3_capture_t * p_capture)
{
    bool b_valid = false;

    if (NULL != p_capture)
    {
        b_valid = p_capture->b_active && (!p_capture->b_failed) &&
                (!p_capture->b_black) && (p_capture->count > 0U);
        p_capture->b_active = false;
        p_capture->b_failed = !b_valid;
    }

    return b_valid;
}

static bool s3_bar_pattern(uint32_t const * p_runs, uint32_t narrow_max,
                           uint32_t wide_min, uint32_t wide_max,
                           uint16_t * p_pattern)
{
    bool b_valid = true;
    uint16_t pattern = 0U;

    for (uint8_t idx = 0U; idx < S3_BAR_ELEMENTS; ++idx)
    {
        uint32_t duration = p_runs[idx];

        /* A nine-element pattern never exceeds 511. */
        pattern = (uint16_t)(pattern << 1U);
        if ((duration >= wide_min) && (duration <= wide_max))
        {
            pattern |= 1U;
        }
        else if ((0U == duration) || (duration > narrow_max))
        {
            b_valid = false;
        }
        else
        {
            /* A narrow run contributes a zero bit. */
        }
    }

    *p_pattern = pattern;
    return b_valid;
}

static uint8_t s3_bar_pattern_distance(uint16_t first, uint16_t second)
{
    /* Patterns are 9-bit, so the XOR and its shifts stay below 512 and the
       distance is at most 9. */
    uint16_t differences = (uint16_t)(first ^ second);
    uint8_t distance = 0U;

    while (0U != differences)
    {
        distance = (uint8_t)(distance + (differences & 1U));
        differences = (uint16_t)(differences >> 1U);
    }

    return distance;
}

bool s3_bar_decode(s3_capture_t const * p_capture,
                   uint32_t narrow_max_us, uint32_t wide_min_us,
                   uint32_t wide_max_us, uint16_t sentinel,
                   s3_symbol_t const * p_symbols, uint8_t symbol_count,
                   char * p_symbol)
{
    uint16_t patterns[3] = {0U};
    uint8_t best_distance = S3_BAR_ELEMENTS + 1U;
    uint8_t best_matches = 0U;
    char decoded = '\0';
    bool b_valid = (NULL != p_capture) && (NULL != p_symbols) &&
                 (NULL != p_symbol) && (narrow_max_us > 0U) &&
                 (wide_min_us > narrow_max_us) &&
                 (wide_max_us >= wide_min_us);

    if (b_valid)
    {
        b_valid = (!p_capture->b_active) && (!p_capture->b_failed) &&
                (S3_BAR_FRAME_RUNS == p_capture->count);
    }

    if (b_valid)
    {
        for (uint8_t idx = 0U; idx < 3U; ++idx)
        {
            bool b_pattern_ok = s3_bar_pattern(&p_capture->runs[idx * 10U],
                                narrow_max_us, wide_min_us, wide_max_us,
                                &patterns[idx]);
            b_valid = b_valid && b_pattern_ok;
        }

        b_valid = b_valid && (patterns[0] == sentinel) &&
                (patterns[2] == sentinel) &&
                (p_capture->runs[9] > 0U) &&
                (p_capture->runs[9] <= narrow_max_us) &&
                (p_capture->runs[19] > 0U) &&
                (p_capture->runs[19] <= narrow_max_us);
    }

    if (b_valid)
    {
        for (uint8_t idx = 0U; idx < symbol_count; ++idx)
        {
            uint8_t const distance = s3_bar_pattern_distance(
                p_symbols[idx].pattern, patterns[1]);

            if (distance < best_distance)
            {
                best_distance = distance;
                best_matches = 1U;
                decoded = p_symbols[idx].symbol;
            }
            else if (distance == best_distance)
            {
                if (best_matches < 2U)
                {
                    ++best_matches;
                }
            }
            else
            {
                /* This symbol is not the closest candidate. */
            }
        }
    }

    b_valid = b_valid && (1U == best_matches)
            && (best_distance <= S3_BAR_SYMBOL_ERROR_MAX)
            && ('\0' != decoded);
    if (b_valid)
    {
        *p_symbol = decoded;
    }

    return b_valid;
}

/** End of file. */

