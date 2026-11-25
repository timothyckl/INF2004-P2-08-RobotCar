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
/*
 * Check of the damaged end in the 24..29-run fallback (F-25, S3 d11).
 * The runs seen at that end must agree with the sentinel copied over
 * them in all but one element: a real damaged end (the recorded 1.5 s
 * pause on the closing bar) disagrees in one. A blip inside the kept 20
 * runs shifts the payload by a bar and a space and makes the seen end
 * disagree in at least two elements, whatever the letter, once at least
 * 7 of its runs are seen (a 27- or 29-run capture). With 5 seen runs a
 * shift at the start end changes only one element, and the recorded
 * 25-run Z disagrees in two, so a 25-run capture is not checked.
 */
#define S3_SCAN_END_MISMATCH_MAX 1U
#define S3_SCAN_END_CHECK_RUNS   7U

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
static bool s3_scan_threshold(uint32_t const * p_runs, uint8_t count,
                              uint32_t * p_narrow_max, uint32_t * p_widest);
static uint8_t s3_scan_mismatches(uint32_t const * p_seen,
                                  uint32_t const * p_copied, uint8_t length,
                                  uint32_t seen_max, uint32_t copied_max);
static bool s3_scan_end_agrees(s3_capture_t const * p_frame,
                               uint32_t const * p_seen,
                               uint32_t const * p_copied, uint8_t length);

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
    uint32_t narrow_max = 0U;
    uint32_t widest = 0U;
    bool b_decoded = false;

    if (S3_BAR_FRAME_RUNS != p_capture->count)
    {
        return false;
    }

    if (s3_scan_threshold(p_capture->runs, p_capture->count, &narrow_max,
                          &widest))
    {
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

    /* The runs seen at the damaged end must agree with the sentinel
     * copied over them (F-25): the end's first runs for the prefix; for
     * the suffix, its last runs, which are the copied sentinel's last
     * elements. count - 20 (4..9) runs were seen there. */
    b_prefix_ok = s3_scan_end_agrees(&prefix,
                                     &p_capture->runs[S3_SCAN_CORE_RUNS],
                                     &prefix.runs[S3_SCAN_CORE_RUNS],
                                     suffix_start)
                  && s3_scan_decode_frame(&prefix, p_cfg, &prefix_symbol);
    b_suffix_ok = s3_scan_end_agrees(&suffix, &p_capture->runs[0],
                                     &suffix.runs[S3_BAR_ELEMENTS
                                                  - suffix_start],
                                     suffix_start)
                  && s3_scan_decode_frame(&suffix, p_cfg, &suffix_symbol);

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

/* The narrow/wide boundary of count runs: halfway between the narrowest
 * and the widest. False if they are too close to tell apart. */
static bool s3_scan_threshold(uint32_t const * p_runs, uint8_t count,
                              uint32_t * p_narrow_max, uint32_t * p_widest)
{
    uint32_t narrowest = UINT32_MAX;
    uint32_t widest = 0U;
    bool b_contrast;

    for (uint8_t idx = 0U; idx < count; ++idx)
    {
        uint32_t const run = p_runs[idx];

        narrowest = (run < narrowest) ? run : narrowest;
        widest = (run > widest) ? run : widest;
    }

    /* Widened products cannot overflow for runs below 2^31 us. */
    b_contrast = ((uint64_t)widest * S3_SCAN_CONTRAST_DEN)
                 >= ((uint64_t)narrowest * S3_SCAN_CONTRAST_NUM);
    /* The mean of two uint32_t values fits uint32_t. */
    *p_narrow_max = (uint32_t)(((uint64_t)narrowest + widest) / 2U);
    *p_widest = widest;

    return b_contrast;
}

/* Elements of p_seen and p_copied (length each) that differ in class at
 * the boundaries seen_max and copied_max. */
static uint8_t s3_scan_mismatches(uint32_t const * p_seen,
                                  uint32_t const * p_copied, uint8_t length,
                                  uint32_t seen_max, uint32_t copied_max)
{
    uint8_t mismatches = 0U;

    for (uint8_t idx = 0U; idx < length; ++idx)
    {
        if ((p_seen[idx] > seen_max) != (p_copied[idx] > copied_max))
        {
            /* At most 9 runs, so the count fits uint8_t. */
            mismatches = (uint8_t)(mismatches + 1U);
        }
    }

    return mismatches;
}

/* Whether the length runs seen at a damaged end (p_seen) agree with the
 * sentinel elements copied over them in the repaired frame p_frame
 * (p_copied), in all but S3_SCAN_END_MISMATCH_MAX elements. Agreement is
 * taken at the frame's boundary (one stretched run, such as a pause) or
 * at each end's own boundary (a change of speed along the code). */
static bool s3_scan_end_agrees(s3_capture_t const * p_frame,
                               uint32_t const * p_seen,
                               uint32_t const * p_copied, uint8_t length)
{
    uint32_t frame_max = 0U;
    uint32_t seen_max = 0U;
    uint32_t copied_max = 0U;
    uint32_t widest = 0U;

    if (length < S3_SCAN_END_CHECK_RUNS)
    {
        return true;
    }

    (void)s3_scan_threshold(p_frame->runs, p_frame->count, &frame_max,
                            &widest);
    (void)s3_scan_threshold(p_seen, length, &seen_max, &widest);
    (void)s3_scan_threshold(p_copied, length, &copied_max, &widest);

    return (s3_scan_mismatches(p_seen, p_copied, length, frame_max,
                               frame_max) <= S3_SCAN_END_MISMATCH_MAX)
           || (s3_scan_mismatches(p_seen, p_copied, length, seen_max,
                                  copied_max) <= S3_SCAN_END_MISMATCH_MAX);
}

/** End of file. */
