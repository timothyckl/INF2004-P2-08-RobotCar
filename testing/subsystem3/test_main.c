/** @file test_main.c Synthetic inputs; these are not robot measurements. */
#include "test_main.h"
#include "s3_ir.h"
#include "s3_bar.h"
#include "s3_nav.h"
#include <stddef.h>
#include <stdio.h>

static unsigned int g_checks = 0U;
static unsigned int g_failures = 0U;

static void check(bool passed, char const * p_name)
{
    ++g_checks;
    if (!passed)
    {
        ++g_failures;
        printf("FAIL: %s\n", p_name);
    }
}

static void test_ir(void)
{
    s3_calibration_t calibration = {0};
    s3_filter_t filter = {0};
    uint8_t mask = 0U;
    s3_line_t const decisions[S3_IR_PATTERNS] =
    {
        S3_LINE_LOST, S3_LINE_LEFT, S3_LINE_STRAIGHT, S3_LINE_UNKNOWN,
        S3_LINE_RIGHT, S3_LINE_UNKNOWN, S3_LINE_UNKNOWN, S3_LINE_JUNCTION
    };

    check(!s3_ir_normalize(&calibration, 0U, &mask), "uncalibrated");
    for (uint8_t black = 0U; black < S3_IR_PATTERNS; ++black)
    {
        check(s3_ir_calibrate(&calibration, black ^ 7U, black),
              "all polarity combinations calibrate");
        for (uint8_t levels = 0U; levels < S3_IR_PATTERNS; ++levels)
        {
            check(s3_ir_normalize(&calibration, levels, &mask),
                  "normalization accepted");
            check(mask == ((levels ^ black) ^ 7U),
                  "all normalized patterns");
        }
    }

    check(!s3_ir_calibrate(&calibration, 0U, 6U), "stuck sensor");
    check(!calibration.valid, "failed recalibration invalidates");
    check(!s3_ir_calibrate(NULL, 0U, 7U), "null calibration");
    check(!s3_ir_normalize(NULL, 0U, &mask), "null normalizer");
    check(!s3_ir_calibrate(&calibration, 8U, 7U), "invalid bits");
    check(!s3_ir_filter(&filter, 1U, 3U), "filter first sample");
    check(!s3_ir_filter(&filter, 1U, 3U), "filter second sample");
    check(s3_ir_filter(&filter, 1U, 3U), "filter settled");
    check(!s3_ir_filter(&filter, 2U, 3U), "glitch not settled");
    check(!filter.ready, "no stale readiness");
    check(!s3_ir_filter(&filter, 2U, 0U), "zero filter requirement");
    for (uint16_t idx = 0U; idx < 300U; ++idx)
    {
        check(s3_ir_filter(&filter, 2U, 1U), "filter saturation");
    }

    for (uint8_t idx = 0U; idx < S3_IR_PATTERNS; ++idx)
    {
        check(s3_ir_line(idx, decisions) == decisions[idx],
              "configured line table");
    }
    check(S3_LINE_UNKNOWN == s3_ir_line(8U, decisions), "invalid mask");
    check(S3_LINE_UNKNOWN == s3_ir_line(0U, NULL), "missing line table");
}

static void feed_symbol(s3_capture_t * p_capture, uint16_t pattern,
                         uint32_t * p_now)
{
    for (uint8_t idx = 0U; idx < S3_BAR_ELEMENTS; ++idx)
    {
        bool wide = (0U != (pattern & (1U << (8U - idx))));

        *p_now += wide ? 300U : 100U;
        check(s3_bar_edge(p_capture, !p_capture->black, *p_now, 500U),
              "symbol edge accepted");
    }
}

static void make_frame(s3_capture_t * p_capture)
{
    uint32_t now = UINT32_MAX - 50U;

    s3_bar_begin(p_capture, now);
    feed_symbol(p_capture, 0x101U, &now);
    now += 100U;
    check(s3_bar_edge(p_capture, true, now, 500U), "first gap");
    feed_symbol(p_capture, 0x103U, &now);
    now += 100U;
    check(s3_bar_edge(p_capture, true, now, 500U), "second gap");
    feed_symbol(p_capture, 0x101U, &now);
    check(s3_bar_finish(p_capture), "finish frame across timer wrap");
}

static void test_barcode(void)
{
    s3_capture_t capture = {0};
    /* Deliberately synthetic patterns, not a claimed printed alphabet. */
    s3_symbol_t const symbols[] = {{0x103U, 'A'}, {0x105U, 'B'}};
    s3_symbol_t const duplicate[] = {{0x103U, 'A'}, {0x103U, 'B'}};
    char decoded = '?';

    make_frame(&capture);
    check(s3_bar_decode(&capture, 150U, 250U, 400U, 0x101U,
                         symbols, 2U, &decoded), "valid framed symbol");
    check('A' == decoded, "decoded payload");
    check(!s3_bar_decode(&capture, 150U, 250U, 400U, 0x100U,
                          symbols, 2U, &decoded), "bad sentinel");
    check(!s3_bar_decode(&capture, 150U, 250U, 400U, 0x101U,
                          duplicate, 2U, &decoded), "ambiguous alphabet");
    check(!s3_bar_decode(&capture, 300U, 250U, 400U, 0x101U,
                          symbols, 2U, &decoded), "invalid thresholds");
    capture.runs[10] = 200U;
    decoded = '?';
    check(!s3_bar_decode(&capture, 150U, 250U, 400U, 0x101U,
                          symbols, 2U, &decoded), "ambiguous duration");
    check('?' == decoded, "no output on decode failure");
    make_frame(&capture);
    capture.runs[9] = 300U;
    check(!s3_bar_decode(&capture, 150U, 250U, 400U, 0x101U,
                          symbols, 2U, &decoded), "invalid gap");
    capture.count = 28U;
    check(!s3_bar_decode(&capture, 150U, 250U, 400U, 0x101U,
                          symbols, 2U, &decoded), "truncated frame");
    s3_bar_begin(&capture, 0U);
    check(!s3_bar_edge(&capture, true, 10U, 500U), "repeated level");
    check(0U == capture.count, "repeated level not stored");
    check(s3_bar_expire(&capture, 501U, 500U), "no-edge timeout");
    check(!s3_bar_finish(&capture), "cannot finish timed-out frame");
    s3_bar_begin(&capture, 0U);
    check(!s3_bar_edge(&capture, false, 0U, 500U), "zero run rejected");
    check(capture.failed, "zero run aborts");
    s3_bar_begin(&capture, 0U);
    for (uint32_t idx = 1U; idx <= S3_BAR_CAPACITY; ++idx)
    {
        check(s3_bar_edge(&capture, !capture.black, idx, 500U),
              "bounded capture");
    }
    check(!s3_bar_edge(&capture, !capture.black, 65U, 500U), "overflow");
    check(capture.failed && (!capture.active), "overflow aborts");
    check(!s3_bar_finish(NULL), "null finish");
    check(!s3_bar_edge(NULL, true, 0U, 1U), "null edge");
}

static void test_navigation(void)
{
    s3_navigation_t navigation = {0};

    check(S3_NAV_LEFT == s3_nav_command('A'), "A maps left");
    check(S3_NAV_RIGHT == s3_nav_command('B'), "B maps right");
    check(S3_NAV_STRAIGHT == s3_nav_command('C'), "C maps straight");
    check(S3_NAV_UTURN == s3_nav_command('D'), "D maps U-turn");
    check(!s3_nav_stage(&navigation, 'Z', 1U, 0U), "invalid command");
    check(s3_nav_stage(&navigation, 'A', 1U, 0U), "stage command");
    check(!s3_nav_stage(&navigation, 'B', 2U, 0U), "no overwrite");
    check(S3_NAV_NONE == s3_nav_take(&navigation, false, true, false,
                                    1U, 100U), "hold before junction");
    check(S3_NAV_NONE == s3_nav_take(&navigation, false, false, true,
                                    2U, 100U), "require authority");
    check(S3_NAV_LEFT == s3_nav_take(&navigation, false, true, true,
                                    3U, 100U), "junction delivery");
    check(S3_NAV_NONE == s3_nav_take(&navigation, false, true, true,
                                    4U, 100U), "one-shot delivery");
    check(!s3_nav_stage(&navigation, 'A', 1U, 4U), "duplicate scan");
    check(s3_nav_stage(&navigation, 'B', 2U, 4U), "new scan");
    check(S3_NAV_NONE == s3_nav_take(&navigation, true, true, true,
                                    5U, 100U), "STOP cancels");
    check(!s3_nav_stage(&navigation, 'B', 2U, 6U), "no stopped replay");
    check(s3_nav_stage(&navigation, 'C', 3U, 6U), "stage before expiry");
    check(S3_NAV_NONE == s3_nav_take(&navigation, false, true, true,
                                    107U, 100U), "expired command");
    check(s3_nav_stage(&navigation, 'D', 4U, UINT32_MAX - 5U),
          "stage across clock wrap");
    check(S3_NAV_UTURN == s3_nav_take(&navigation, false, true, true,
                                     5U, 100U), "age across clock wrap");
    check(!s3_nav_stage(NULL, 'A', 0U, 0U), "null navigation");
    check(S3_NAV_NONE == s3_nav_take(NULL, false, true, true, 0U, 1U),
          "null take");
}

static void test_pipeline(void)
{
    s3_capture_t capture = {0};
    s3_navigation_t navigation = {0};
    s3_symbol_t const symbols[] = {{0x103U, 'A'}};
    char decoded = '\0';

    make_frame(&capture);
    bool valid = s3_bar_decode(&capture, 150U, 250U, 400U, 0x101U,
                                symbols, 1U, &decoded);
    check(valid, "pipeline complete barcode");
    if (valid)
    {
        check(s3_nav_stage(&navigation, decoded, 7U, 10U),
              "pipeline stages decoded symbol");
    }

    check(S3_NAV_NONE == s3_nav_take(&navigation, false, true, false,
                                    20U, 100U), "pipeline waits");
    check(S3_NAV_LEFT == s3_nav_take(&navigation, false, true, true,
                                    30U, 100U), "pipeline navigation request");
}

int main(void)
{
    test_ir();
    test_barcode();
    test_navigation();
    test_pipeline();
    printf("%u checks, %u failures\n", g_checks, g_failures);
    return (0U == g_failures) ? 0 : 1;
}

/** End of file. */

