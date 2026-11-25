/** @file s3_scan.c Capture framing and adaptive windows; no GPIO or RTOS. */
#include "s3_scan.h"
#include <stddef.h>

/* Wide runs must be at least 1.5 times the narrow ones to be told apart
 * (Code 39 prints wide:narrow between 2:1 and 3:1). */
#define S3_SCAN_CONTRAST_NUM    3U
#define S3_SCAN_CONTRAST_DEN    2U
#define S3_SCAN_PARTIAL_MIN_RUNS 24U
#define S3_SCAN_CORE_RUNS        ((S3_BAR_ELEMENTS * 2U) + 2U)
/* One stray mark (a bar and a space) beside the code, as seen on the car
 * on 2026-11-17 (testing/subsystem3/TEST_LOG.md). */
#define S3_SCAN_STRAY_RUNS       2U

static bool s3_scan_decode(s3_capture_t const * p_capture,
                           s3_scan_cfg_t const * p_cfg, char * p_symbol);
static bool s3_scan_decode_29(s3_capture_t const * p_capture,
                              s3_scan_cfg_t const * p_cfg, char * p_symbol);
static bool s3_scan_decode_31(s3_capture_t const * p_capture,
                              s3_scan_cfg_t const * p_cfg, char * p_symbol);
static bool s3_scan_decode_frame(s3_capture_t const * p_capture,
                                 s3_scan_cfg_t const * p_cfg,
                                 char * p_symbol);
static bool s3_scan_decode_partial(s3_capture_t const * p_capture,
                                   s3_scan_cfg_t const * p_cfg,
                                   char * p_symbol);

void s3_scan_init(s3_scan_t * p_scan)
{
    if (NULL != p_scan)
    {
        s3_scan_t const zero = {0};

        *p_scan = zero;
    }
}

void s3_scan_abort(s3_scan_t * p_scan)
{
    if (NULL != p_scan)
    {
        p_scan->capture.b_active = false;
        p_scan->capture.b_failed = false;
    }
}

s3_scan_result_t s3_scan_edge(s3_scan_t * p_scan, s3_scan_cfg_t const * p_cfg,
                              bool b_black, uint32_t now_us)
{
    s3_scan_result_t result = S3_SCAN_NONE;

    if ((NULL == p_scan) || (NULL == p_cfg))
    {
        return result;
    }

    if (p_scan->capture.b_active)
    {
        (void)s3_bar_edge(&p_scan->capture, b_black, now_us, p_cfg->run_max_us);

        if ((!p_scan->capture.b_active) && p_scan->capture.b_failed)
        {
            result = S3_SCAN_ERROR;     /* overflow or an over-long run */
        }
    }
    else if (b_black && (!p_scan->b_black))
    {
        s3_bar_begin(&p_scan->capture, now_us);
    }
    else
    {
        /* White while idle, or a repeated level. */
    }

    p_scan->b_black = b_black;

    return result;
}

s3_scan_result_t s3_scan_poll(s3_scan_t * p_scan, s3_scan_cfg_t const * p_cfg,
                              uint32_t now_us, char * p_symbol)
{
    s3_scan_result_t result = S3_SCAN_NONE;

    if ((NULL == p_scan) || (NULL == p_cfg) || (NULL == p_symbol)
        || (!p_scan->capture.b_active))
    {
        return result;
    }

    uint32_t const idle_us = now_us - p_scan->capture.last_us;

    if ((!p_scan->capture.b_black) && (idle_us >= p_cfg->quiet_us))
    {
        uint8_t const count = p_scan->capture.count;

        if (!s3_bar_finish(&p_scan->capture))
        {
            result = S3_SCAN_ERROR;
        }
        else if (1U == count)
        {
            result = S3_SCAN_SINGLE_BAR;
        }
        else if (s3_scan_decode(&p_scan->capture, p_cfg, p_symbol))
        {
            result = S3_SCAN_DECODED;
        }
        else
        {
            result = S3_SCAN_ERROR;
        }
    }
    else if (s3_bar_expire(&p_scan->capture, now_us, p_cfg->run_max_us))
    {
        result = S3_SCAN_ERROR;         /* black for too long */
    }
    else
    {
        /* Capture still in progress. */
    }

    return result;
}

static bool s3_scan_decode(s3_capture_t const * p_capture,
                           s3_scan_cfg_t const * p_cfg, char * p_symbol)
{
    bool b_decoded = false;

    if (S3_BAR_FRAME_RUNS == p_capture->count)
    {
        b_decoded = s3_scan_decode_29(p_capture, p_cfg, p_symbol);
    }
    else if ((p_capture->count >= S3_SCAN_PARTIAL_MIN_RUNS)
             && (p_capture->count < S3_BAR_FRAME_RUNS))
    {
        b_decoded = s3_scan_decode_partial(p_capture, p_cfg, p_symbol);
    }
    else if ((S3_BAR_FRAME_RUNS + S3_SCAN_STRAY_RUNS) == p_capture->count)
    {
        b_decoded = s3_scan_decode_31(p_capture, p_cfg, p_symbol);
    }
    else
    {
        /* Less than 80 percent cannot use the bounded partial fallback. */
    }

    return b_decoded;
}

/* A complete 29-run frame; if one sentinel is damaged, the partial
 * fallback rebuilds it from the intact one. */
static bool s3_scan_decode_29(s3_capture_t const * p_capture,
                              s3_scan_cfg_t const * p_cfg, char * p_symbol)
{
    return s3_scan_decode_frame(p_capture, p_cfg, p_symbol)
           || s3_scan_decode_partial(p_capture, p_cfg, p_symbol);
}

/*
 * A 31-run capture holds one extra bar and space. Its shortest run tells
 * where they came from (F-18):
 *  - at either end, a stray mark beside the code (seen on the car on
 *    2026-11-17): the 29 runs without it and its space are decoded;
 *  - inside the code, a noise blip that split one element in three: the
 *    element is rebuilt from the blip and its two neighbours.
 * Decoding a window shifted past a blip instead can match a different
 * letter exactly (S3-U03), so the window is chosen, never searched for.
 */
static bool s3_scan_decode_31(s3_capture_t const * p_capture,
                              s3_scan_cfg_t const * p_cfg, char * p_symbol)
{
    uint8_t const last = (uint8_t)(S3_BAR_FRAME_RUNS + S3_SCAN_STRAY_RUNS
                                   - 1U);
    s3_capture_t window = *p_capture;
    uint8_t shortest = 0U;

    /* On a tie the first run is taken; either choice is then checked by
     * the full frame decode. */
    for (uint8_t idx = 1U; idx <= last; ++idx)
    {
        if (p_capture->runs[idx] < p_capture->runs[shortest])
        {
            shortest = idx;
        }
    }

    for (uint8_t idx = 0U; idx < S3_BAR_FRAME_RUNS; ++idx)
    {
        if (0U == shortest)
        {
            window.runs[idx] = p_capture->runs[idx + S3_SCAN_STRAY_RUNS];
        }
        else if ((last == shortest) || (idx < (shortest - 1U)))
        {
            window.runs[idx] = p_capture->runs[idx];
        }
        else if (idx == (shortest - 1U))
        {
            /* Three runs of at most UINT32_MAX us each fit in 64 bits;
             * the sum saturates rather than wraps. */
            uint64_t const merged = (uint64_t)p_capture->runs[idx]
                                    + p_capture->runs[idx + 1U]
                                    + p_capture->runs[idx + 2U];

            window.runs[idx] = (merged > UINT32_MAX) ? UINT32_MAX
                                                     : (uint32_t)merged;
        }
        else
        {
            window.runs[idx] = p_capture->runs[idx + S3_SCAN_STRAY_RUNS];
        }
    }
    window.count = S3_BAR_FRAME_RUNS;

    return s3_scan_decode_29(&window, p_cfg, p_symbol);
}

static bool s3_scan_decode_frame(s3_capture_t const * p_capture,
                                 s3_scan_cfg_t const * p_cfg,
                                 char * p_symbol)
{
    uint32_t narrowest = UINT32_MAX;
    uint32_t widest = 0U;
    bool b_decoded = false;

    if (S3_BAR_FRAME_RUNS != p_capture->count)
    {
        return false;
    }

    for (uint8_t idx = 0U; idx < p_capture->count; ++idx)
    {
        uint32_t const run = p_capture->runs[idx];

        narrowest = (run < narrowest) ? run : narrowest;
        widest = (run > widest) ? run : widest;
    }

    /* Widened products cannot overflow for runs below 2^31 us. */
    if (((uint64_t)widest * S3_SCAN_CONTRAST_DEN)
        >= ((uint64_t)narrowest * S3_SCAN_CONTRAST_NUM))
    {
        uint32_t const narrow_max = (uint32_t)(((uint64_t)narrowest + widest)
                                               / 2U);
        s3_capture_t reversed = *p_capture;

        b_decoded = s3_bar_decode(p_capture, narrow_max, narrow_max + 1U,
                                widest, p_cfg->sentinel, p_cfg->p_symbols,
                                p_cfg->symbol_count, p_symbol);

        if (!b_decoded)
        {
            /* Driven past the code the other way: same runs, reversed. */
            for (uint8_t idx = 0U; idx < p_capture->count; ++idx)
            {
                reversed.runs[idx] =
                    p_capture->runs[p_capture->count - 1U - idx];
            }

            b_decoded = s3_bar_decode(&reversed, narrow_max, narrow_max + 1U,
                                    widest, p_cfg->sentinel,
                                    p_cfg->p_symbols, p_cfg->symbol_count,
                                    p_symbol);
        }
    }

    return b_decoded;
}

static bool s3_scan_decode_partial(s3_capture_t const * p_capture,
                                   s3_scan_cfg_t const * p_cfg,
                                   char * p_symbol)
{
    s3_capture_t prefix = *p_capture;
    s3_capture_t suffix = *p_capture;
    /* count is 24..29 on this path, so count - 20 is 4..9. */
    uint8_t const suffix_start =
        (uint8_t)(p_capture->count - S3_SCAN_CORE_RUNS);
    char prefix_symbol = '\0';
    char suffix_symbol = '\0';
    bool b_prefix_ok;
    bool b_suffix_ok;

    /* Repair only the terminal sentinel. The retained 20-run core is still
     * sentinel, gap, payload, gap, so no payload element is guessed. */
    for (uint8_t idx = 0U; idx < S3_BAR_ELEMENTS; ++idx)
    {
        prefix.runs[S3_SCAN_CORE_RUNS + idx] = p_capture->runs[idx];
        suffix.runs[idx] = p_capture->runs[
            suffix_start + S3_BAR_ELEMENTS + 2U + idx];
    }
    prefix.count = S3_BAR_FRAME_RUNS;

    /* A retained suffix is gap, payload, gap, sentinel. Prepend a copy of
     * its intact sentinel to form a complete frame for normal validation. */
    for (uint8_t idx = 0U; idx < S3_SCAN_CORE_RUNS; ++idx)
    {
        suffix.runs[S3_BAR_ELEMENTS + idx] =
            p_capture->runs[suffix_start + idx];
    }
    suffix.count = S3_BAR_FRAME_RUNS;

    b_prefix_ok = s3_scan_decode_frame(&prefix, p_cfg, &prefix_symbol);
    b_suffix_ok = s3_scan_decode_frame(&suffix, p_cfg, &suffix_symbol);

    if (b_prefix_ok && b_suffix_ok && (prefix_symbol != suffix_symbol))
    {
        return false;
    }
    if (b_prefix_ok || b_suffix_ok)
    {
        *p_symbol = b_prefix_ok ? prefix_symbol : suffix_symbol;
        return true;
    }

    return false;
}

/** End of file. */
