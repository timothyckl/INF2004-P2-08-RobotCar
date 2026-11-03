/** @file s3_scan.c Capture framing and adaptive windows; no GPIO or RTOS. */
#include "s3_scan.h"
#include <stddef.h>

/* Wide runs must be at least 1.5 times the narrow ones to be told apart
 * (Code 39 prints wide:narrow between 2:1 and 3:1). */
#define S3_SCAN_CONTRAST_NUM    3U
#define S3_SCAN_CONTRAST_DEN    2U

static bool s3_scan_decode(s3_capture_t const * p_capture,
                           s3_scan_cfg_t const * p_cfg, char * p_symbol);

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
        p_scan->capture.active = false;
        p_scan->capture.failed = false;
    }
}

s3_scan_result_t s3_scan_edge(s3_scan_t * p_scan, s3_scan_cfg_t const * p_cfg,
                              bool black, uint32_t now_us)
{
    s3_scan_result_t result = S3_SCAN_NONE;

    if ((NULL == p_scan) || (NULL == p_cfg))
    {
        return result;
    }

    if (p_scan->capture.active)
    {
        (void)s3_bar_edge(&p_scan->capture, black, now_us, p_cfg->run_max_us);

        if ((!p_scan->capture.active) && p_scan->capture.failed)
        {
            result = S3_SCAN_ERROR;     /* overflow or an over-long run */
        }
    }
    else if (black && (!p_scan->black))
    {
        s3_bar_begin(&p_scan->capture, now_us);
    }
    else
    {
        /* White while idle, or a repeated level. */
    }

    p_scan->black = black;

    return result;
}

s3_scan_result_t s3_scan_poll(s3_scan_t * p_scan, s3_scan_cfg_t const * p_cfg,
                              uint32_t now_us, char * p_symbol)
{
    s3_scan_result_t result = S3_SCAN_NONE;

    if ((NULL == p_scan) || (NULL == p_cfg) || (NULL == p_symbol)
        || (!p_scan->capture.active))
    {
        return result;
    }

    uint32_t const idle_us = now_us - p_scan->capture.last_us;

    if ((!p_scan->capture.black) && (idle_us >= p_cfg->quiet_us))
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
    uint32_t narrowest = UINT32_MAX;
    uint32_t widest = 0U;
    bool decoded = false;

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

        decoded = s3_bar_decode(p_capture, narrow_max, narrow_max + 1U,
                                widest, p_cfg->sentinel, p_cfg->p_symbols,
                                p_cfg->symbol_count, p_symbol);

        if (!decoded)
        {
            /* Driven past the code the other way: same runs, reversed. */
            for (uint8_t idx = 0U; idx < p_capture->count; ++idx)
            {
                reversed.runs[idx] =
                    p_capture->runs[p_capture->count - 1U - idx];
            }

            decoded = s3_bar_decode(&reversed, narrow_max, narrow_max + 1U,
                                    widest, p_cfg->sentinel,
                                    p_cfg->p_symbols, p_cfg->symbol_count,
                                    p_symbol);
        }
    }

    return decoded;
}

/** End of file. */
