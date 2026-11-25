/** @file test_main.c Synthetic inputs; these are not robot measurements. */
#include "test_main.h"
#include "s3_ir.h"
#include "s3_bar.h"
#include "s3_barcode.h"
#include "s3_code39.h"
#include "s3_nav.h"
#include "s3_scan.h"
#include <stddef.h>
#include <stdio.h>

/* Scan limits used by these synthetic cases. They equal the provisional
 * values on feature/subsystem3-ir-line-barcode; the firmware values are
 * owned by the Line Sensor task and still require S3 hardware validation. */
#define TEST_BAR_RUN_MAX_US     800000U
#define TEST_BAR_QUIET_US       700000U

static unsigned int g_checks = 0U;
static unsigned int g_failures = 0U;
static unsigned int g_known = 0U;

static void check(bool passed, char const * p_name)
{
    ++g_checks;
    if (!passed)
    {
        ++g_failures;
        printf("FAIL: %s\n", p_name);
    }
}

/* A property the firmware is known not to meet yet, recorded with its
 * finding ID (docs/software_quality_review.md). It is reported on every
 * run but does not fail the suite; if the firmware is corrected it
 * reports FIXED so that it can become an ordinary check. */
static void known_defect(bool passed, char const * p_id, char const * p_name)
{
    if (passed)
    {
        printf("FIXED (%s): %s\n", p_id, p_name);
    }
    else
    {
        ++g_known;
        printf("KNOWN DEFECT (%s): %s\n", p_id, p_name);
    }
}

static void test_ir(void)
{
    s3_filter_t filter = {0};
    s3_ir_adaptive_t adaptive = {0};
    s3_ir_adaptive_config_t const adaptive_config =
    {
        136U, 96U, 168U, 32U, 20U
    };
    s3_ir_adaptive_config_t const invalid_config =
    {
        136U, 170U, 168U, 32U, 20U
    };
    s3_line_t const decisions[S3_IR_PATTERNS] =
    {
        S3_LINE_UNKNOWN, S3_LINE_LEFT,
        S3_LINE_RIGHT, S3_LINE_STRAIGHT
    };

    check(!s3_ir_filter(&filter, 1U, 3U), "filter first sample");
    check(!s3_ir_filter(&filter, 1U, 3U), "filter second sample");
    check(s3_ir_filter(&filter, 1U, 3U), "filter settled");
    check(!s3_ir_filter(&filter, 2U, 3U), "glitch not settled");
    check(!filter.b_ready, "no stale readiness");
    check(!s3_ir_filter(&filter, 2U, 0U), "zero filter requirement");
    for (uint16_t idx = 0U; idx < 300U; ++idx)
    {
        check(s3_ir_filter(&filter, 2U, 1U), "filter saturation");
    }

    check(!s3_ir_adaptive_init(NULL, &adaptive_config),
          "adaptive null state rejected");
    check(!s3_ir_adaptive_init(&adaptive, NULL),
          "adaptive null config rejected");
    check(!s3_ir_adaptive_init(&adaptive, &invalid_config),
          "adaptive invalid white bounds rejected");
    check(s3_ir_adaptive_init(&adaptive, &adaptive_config),
          "adaptive configuration accepted");
    check(168U == adaptive.threshold, "adaptive initial threshold");
    check(0U == s3_ir_adaptive_strength(NULL, 1500U),
          "null adaptive strength rejected");
    check(0U == s3_ir_adaptive_strength(&adaptive, 136U),
          "invalid black reference rejected");
    for (uint8_t idx = 0U; idx < 16U; ++idx)
    {
        check(!s3_ir_adaptive_update(&adaptive, 136U),
              "steady white remains white");
    }
    check(s3_ir_adaptive_update(&adaptive, 206U),
          "filtered faded patch becomes black");
    check(s3_ir_adaptive_update(&adaptive, 206U),
          "filtered faded patch remains black");
    check((s3_ir_adaptive_strength(&adaptive, 1500U) > 0U)
          && (s3_ir_adaptive_strength(&adaptive, 1500U) < 10U),
          "faded patch gives a gentle steering strength");
    uint16_t const black_baseline = adaptive.white_baseline;

    for (uint8_t idx = 0U; idx < 16U; ++idx)
    {
        check(s3_ir_adaptive_update(&adaptive, 206U),
              "steady black remains black");
    }
    check(black_baseline == adaptive.white_baseline,
          "black cannot alter white baseline");
    check(s3_ir_adaptive_update(&adaptive, 136U),
          "hysteresis holds first white sample");
    check(!s3_ir_adaptive_update(&adaptive, 136U),
          "hysteresis releases on stable white");
    for (uint8_t idx = 0U; idx < 80U; ++idx)
    {
        check(!s3_ir_adaptive_update(&adaptive, 154U),
              "elevated white remains white");
    }
    check((adaptive.white_baseline > black_baseline)
          && (adaptive.white_baseline <= adaptive_config.maximum_white),
          "white baseline adapts within bounds");
    check(!s3_ir_adaptive_update(&adaptive, 4096U),
          "out-of-range ADC sample rejected");
    adaptive.filtered = 1500U;
    check(100U == s3_ir_adaptive_strength(&adaptive, 1500U),
          "reference black gives full steering strength");
    adaptive.filtered = S3_IR_ADC_MAX;
    check(100U == s3_ir_adaptive_strength(&adaptive, 1500U),
          "steering strength saturates at one hundred");

    for (uint8_t idx = 0U; idx < S3_IR_PATTERNS; ++idx)
    {
        check(s3_ir_line(idx, decisions) == decisions[idx],
              "configured line table");
    }
    check(S3_LINE_UNKNOWN == s3_ir_line(4U, decisions), "invalid mask");
    check(S3_LINE_UNKNOWN == s3_ir_line(0U, NULL), "missing line table");
    check(S3_LINE_STRAIGHT == s3_ir_follow_decision(0U),
          "white-white drives straight");
    check(S3_LINE_LEFT == s3_ir_follow_decision(S3_IR_LEFT_MASK),
          "left black requests left correction");
    check(S3_LINE_RIGHT == s3_ir_follow_decision(S3_IR_RIGHT_MASK),
          "right black requests right correction");
    check(S3_LINE_JUNCTION == s3_ir_follow_decision(3U),
          "black-black reports a crossing");
    check(S3_LINE_UNKNOWN == s3_ir_follow_decision(4U),
          "invalid follow mask rejected");
}

static void feed_symbol(s3_capture_t * p_capture, uint16_t pattern,
                         uint32_t * p_now)
{
    for (uint8_t idx = 0U; idx < S3_BAR_ELEMENTS; ++idx)
    {
        bool wide = (0U != (pattern & (1U << (8U - idx))));

        *p_now += wide ? 300U : 100U;
        check(s3_bar_edge(p_capture, !p_capture->b_black, *p_now, 500U),
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
    check(capture.b_failed, "zero run aborts");
    s3_bar_begin(&capture, 0U);
    for (uint32_t idx = 1U; idx <= S3_BAR_CAPACITY; ++idx)
    {
        check(s3_bar_edge(&capture, !capture.b_black, idx, 500U),
              "bounded capture");
    }
    check(!s3_bar_edge(&capture, !capture.b_black, 65U, 500U), "overflow");
    check(capture.b_failed && (!capture.b_active), "overflow aborts");
    check(!s3_bar_finish(NULL), "null finish");
    check(!s3_bar_edge(NULL, true, 0U, 1U), "null edge");
}

static void test_navigation(void)
{
    check(S3_NAV_LEFT == s3_nav_command('A'), "A maps left");
    check(S3_NAV_RIGHT == s3_nav_command('B'), "B maps right");
    check(S3_NAV_STRAIGHT == s3_nav_command('C'), "C maps straight");
    check(S3_NAV_UTURN == s3_nav_command('D'), "D maps U-turn");
    check(S3_NAV_NONE == s3_nav_command('F'),
          "F is unmapped unless S3_NAV_F is set");
    check(S3_NAV_NONE == s3_nav_command('Z'), "other letters are unmapped");

    /* Run-time letters (S1 decision 6). */
    check(s3_nav_set_letter(S3_NAV_LEFT, 'G'), "set G left");
    check(S3_NAV_LEFT == s3_nav_command('G'), "G maps left");
    check(S3_NAV_NONE == s3_nav_command('A'), "A no longer left");
    check(s3_nav_set_letter(S3_NAV_RIGHT, 'G'), "set G right");
    check(S3_NAV_RIGHT == s3_nav_command('G'), "G moved to right");
    check(S3_NAV_NONE == s3_nav_command('B'), "B no longer right");
    check(S3_NAV_STRAIGHT == s3_nav_command('C'), "C untouched");
    check(!s3_nav_set_letter(S3_NAV_NONE, 'H'), "NONE rejected");
    check(!s3_nav_set_letter((s3_command_t)5, 'H'), "code 5 rejected");
    check(!s3_nav_set_letter(S3_NAV_LEFT, 'a'), "lower case rejected");
    check(!s3_nav_set_letter(S3_NAV_LEFT, '['), "past Z rejected");
    check(S3_NAV_NONE == s3_nav_command('H'), "rejects change nothing");
    check(S3_NAV_NONE == s3_nav_command('@'), "below A unmapped");
    check(s3_nav_set_letter(S3_NAV_LEFT, 'A') &&
          s3_nav_set_letter(S3_NAV_RIGHT, 'B') &&
          (S3_NAV_NONE == s3_nav_command('G')) &&
          (S3_NAV_LEFT == s3_nav_command('A')) &&
          (S3_NAV_RIGHT == s3_nav_command('B')), "boot map restored");
}

static void test_pipeline(void)
{
    s3_capture_t capture = {0};
    s3_symbol_t const symbols[] = {{0x103U, 'A'}};
    char decoded = '\0';

    make_frame(&capture);
    bool valid = s3_bar_decode(&capture, 150U, 250U, 400U, 0x101U,
                                symbols, 1U, &decoded);
    check(valid, "pipeline complete barcode");
    check(valid && (S3_NAV_LEFT == s3_nav_command(decoded)),
          "pipeline decoded letter maps to its command");
}

static s3_scan_cfg_t scan_cfg(void)
{
    s3_scan_cfg_t cfg = {TEST_BAR_RUN_MAX_US, TEST_BAR_QUIET_US,
                         S3_CODE39_SENTINEL, g_s3_code39_alphabet,
                         S3_CODE39_SYMBOL_COUNT};

    return cfg;
}

/* Feed one 9-element character starting on a bar: one edge per element,
 * at its end, to the opposite level. */
static void feed_char(s3_scan_t * p_scan, s3_scan_cfg_t const * p_cfg,
                      uint16_t pattern, uint32_t narrow, uint32_t wide,
                      bool * p_black, uint32_t * p_now)
{
    for (uint8_t idx = 0U; idx < S3_BAR_ELEMENTS; ++idx)
    {
        bool const b_wide = (0U != (pattern & (1U << (8U - idx))));

        *p_now += b_wide ? wide : narrow;
        *p_black = !*p_black;
        (void)s3_scan_edge(p_scan, p_cfg, *p_black, *p_now);
    }
}

/* Poll until a capture finishes or 100 sample periods pass. */
static s3_scan_result_t scan_settle(s3_scan_t * p_scan,
                                    s3_scan_cfg_t const * p_cfg,
                                    uint32_t step_us, uint32_t * p_now,
                                    char * p_out)
{
    s3_scan_result_t result = S3_SCAN_NONE;

    for (uint8_t tick = 0U; (tick < 100U) && (S3_SCAN_NONE == result);
         ++tick)
    {
        *p_now += step_us;
        result = s3_scan_poll(p_scan, p_cfg, *p_now, p_out);
    }

    return result;
}

/* Drive one framed code past the sensor; reverse feeds it backwards. */
static s3_scan_result_t feed_code(s3_scan_t * p_scan, uint16_t symbol,
                                  uint32_t narrow, uint32_t wide,
                                  bool b_reverse, uint32_t * p_now,
                                  char * p_out)
{
    s3_scan_cfg_t const cfg = scan_cfg();
    uint16_t const order[3] =
    {
        S3_CODE39_SENTINEL, symbol, S3_CODE39_SENTINEL
    };
    bool black = true;

    (void)s3_scan_edge(p_scan, &cfg, true, *p_now);     /* first bar */

    for (uint8_t ch = 0U; ch < 3U; ++ch)
    {
        uint16_t pattern = order[ch];

        if (b_reverse)
        {
            uint16_t flipped = 0U;

            pattern = order[2U - ch];
            for (uint8_t bit = 0U; bit < S3_BAR_ELEMENTS; ++bit)
            {
                flipped = (uint16_t)(flipped << 1U);
                flipped = (uint16_t)(flipped | ((pattern >> bit) & 1U));
            }
            pattern = flipped;
        }

        feed_char(p_scan, &cfg, pattern, narrow, wide, &black, p_now);

        if (ch < 2U)
        {
            *p_now += narrow;               /* inter-character gap */
            black = true;
            (void)s3_scan_edge(p_scan, &cfg, true, *p_now);
        }
    }

    return scan_settle(p_scan, &cfg, 10000U, p_now, p_out);
}

/* Replay the 25-run smaller-Z capture recorded on the barcode branch. Its
 * first 20 runs hold sentinel, gap, payload and gap; only the terminal
 * sentinel is damaged. */
static s3_scan_result_t feed_partial_z(s3_scan_t * p_scan, bool b_reverse,
                                       uint8_t run_count, uint32_t * p_now,
                                       char * p_out)
{
    uint32_t const runs[25] =
    {
        100000U, 360000U, 80000U, 120000U, 300000U,
        120000U, 280000U, 140000U, 60000U, 160000U,
        60000U, 320000U, 300000U, 100000U, 300000U,
        120000U, 80000U, 120000U, 80000U, 120000U,
        100000U, 200000U, 660000U, 20000U, 560000U
    };
    uint8_t const last = (uint8_t)((sizeof(runs) / sizeof(runs[0])) - 1U);
    s3_scan_cfg_t const cfg = scan_cfg();
    bool black = true;

    (void)s3_scan_edge(p_scan, &cfg, true, *p_now);
    for (uint8_t idx = 0U; idx < run_count; ++idx)
    {
        uint8_t const source = b_reverse ? (uint8_t)(last - idx) : idx;

        *p_now += runs[source];
        black = !black;
        (void)s3_scan_edge(p_scan, &cfg, black, *p_now);
    }

    return scan_settle(p_scan, &cfg, 10000U, p_now, p_out);
}

/* Replay recorded run durations in ms, first run black. */
static s3_scan_result_t feed_runs_ms(s3_scan_t * p_scan,
                                     uint16_t const * p_runs_ms,
                                     uint8_t count, uint32_t * p_now,
                                     char * p_out)
{
    s3_scan_cfg_t const cfg = scan_cfg();
    bool black = true;

    (void)s3_scan_edge(p_scan, &cfg, true, *p_now);
    for (uint8_t idx = 0U; idx < count; ++idx)
    {
        *p_now += (uint32_t)p_runs_ms[idx] * 1000U;
        black = !black;
        (void)s3_scan_edge(p_scan, &cfg, black, *p_now);
    }

    return scan_settle(p_scan, &cfg, 10000U, p_now, p_out);
}

/* Drive one framed code past the sensor while the speed changes steadily:
 * run idx of the 29 lasts its nominal time scaled by a factor that moves
 * linearly from 1000 per mille (first run) to end_permille (last run).
 * Above 1000 the car slows down over the code; below, it speeds up. */
static s3_scan_result_t feed_code_ramp(s3_scan_t * p_scan, uint16_t symbol,
                                       uint32_t narrow, uint32_t wide,
                                       uint32_t end_permille,
                                       uint32_t * p_now, char * p_out)
{
    s3_scan_cfg_t const cfg = scan_cfg();
    uint16_t const order[3] =
    {
        S3_CODE39_SENTINEL, symbol, S3_CODE39_SENTINEL
    };
    uint32_t runs[S3_BAR_FRAME_RUNS];
    uint8_t count = 0U;
    bool black = true;

    for (uint8_t ch = 0U; ch < 3U; ++ch)
    {
        for (uint8_t idx = 0U; idx < S3_BAR_ELEMENTS; ++idx)
        {
            bool const b_wide = (0U != (order[ch] & (1U << (8U - idx))));

            runs[count] = b_wide ? wide : narrow;
            ++count;
        }
        if (ch < 2U)
        {
            runs[count] = narrow;           /* inter-character gap */
            ++count;
        }
    }

    (void)s3_scan_edge(p_scan, &cfg, true, *p_now);
    for (uint8_t idx = 0U; idx < count; ++idx)
    {
        /* Linear in idx; all terms stay far below 2^32 for these inputs. */
        uint32_t const scale = (end_permille >= 1000U)
            ? (1000U + (((end_permille - 1000U) * idx)
                        / (S3_BAR_FRAME_RUNS - 1U)))
            : (1000U - (((1000U - end_permille) * idx)
                        / (S3_BAR_FRAME_RUNS - 1U)));

        *p_now += (runs[idx] * scale) / 1000U;
        black = !black;
        (void)s3_scan_edge(p_scan, &cfg, black, *p_now);
    }

    return scan_settle(p_scan, &cfg, 10000U, p_now, p_out);
}

/*
 * S3-U02: decoder robustness (post-demo test, 2026-11-25). Two properties
 * the course depends on, checked over every letter A..Z:
 *  - one element misread (narrow seen as wide or the reverse) is either
 *    corrected to the printed letter or rejected, never a different letter;
 *  - a steady speed change across one code, with a 2.5:1 print (100 ms
 *    narrow, 250 ms wide), decodes every letter while the last run is
 *    0.4 to 2.5 times the first (the sentinel rebuild and the one-element
 *    correction absorb the damaged end). At 0.3 and 3.5 times, letters are
 *    lost (probed on 2026-11-25: 9/26 and 0/26 decoded); a lost letter
 *    must be a rejection, never a wrong letter.
 */
static void test_scan_robustness(void)
{
    s3_scan_t scan;
    uint32_t now = 9000000U;
    uint32_t const ends[4] = {400U, 2500U, 300U, 3500U};
    bool b_never_wrong = true;
    bool b_some_corrected = false;
    bool b_some_rejected = false;

    s3_scan_init(&scan);
    for (uint8_t idx = 0U; idx < S3_CODE39_SYMBOL_COUNT; ++idx)
    {
        for (uint8_t bit = 0U; bit < S3_BAR_ELEMENTS; ++bit)
        {
            uint16_t const damaged = (uint16_t)(
                g_s3_code39_alphabet[idx].pattern ^ (1U << bit));
            char symbol = '\0';
            s3_scan_result_t const result = feed_code(
                &scan, damaged, 2000U, 5000U, false, &now, &symbol);

            if (S3_SCAN_DECODED == result)
            {
                b_never_wrong = b_never_wrong
                    && (g_s3_code39_alphabet[idx].symbol == symbol);
                b_some_corrected = true;
            }
            else
            {
                b_some_rejected = true;
            }
        }
    }
    check(b_never_wrong, "one misread element never gives a wrong letter");
    check(b_some_corrected, "one misread element can be corrected");
    check(b_some_rejected, "an ambiguous misread element is rejected");

    for (uint8_t end = 0U; end < 4U; ++end)
    {
        bool b_all_decoded = true;
        bool b_no_wrong = true;

        for (uint8_t idx = 0U; idx < S3_CODE39_SYMBOL_COUNT; ++idx)
        {
            char symbol = '\0';
            s3_scan_result_t const result = feed_code_ramp(
                &scan, g_s3_code39_alphabet[idx].pattern, 100000U, 250000U,
                ends[end], &now, &symbol);
            bool const b_right = (S3_SCAN_DECODED == result)
                && (g_s3_code39_alphabet[idx].symbol == symbol);

            b_all_decoded = b_all_decoded && b_right;
            b_no_wrong = b_no_wrong
                && ((S3_SCAN_DECODED != result) || b_right);
        }
        if (end < 2U)
        {
            check(b_all_decoded, "steady speed change within limits decodes");
        }
        check(b_no_wrong, "speed change never gives a wrong letter");
    }
}

/* Build the 29 runs of one framed code: sentinel, gap, symbol, gap,
 * sentinel, with the given narrow and wide durations. */
static uint8_t make_runs(uint32_t * p_runs, uint16_t first, uint16_t symbol,
                         uint16_t last, uint32_t narrow, uint32_t wide)
{
    uint16_t const order[3] = {first, symbol, last};
    uint8_t count = 0U;

    for (uint8_t ch = 0U; ch < 3U; ++ch)
    {
        for (uint8_t idx = 0U; idx < S3_BAR_ELEMENTS; ++idx)
        {
            p_runs[count] = (0U != (order[ch] & (1U << (8U - idx))))
                            ? wide : narrow;
            ++count;
        }
        if (ch < 2U)
        {
            p_runs[count] = narrow;
            ++count;
        }
    }

    return count;
}

/* Feed run durations in microseconds, first run black. */
static s3_scan_result_t feed_runs_us(s3_scan_t * p_scan,
                                     uint32_t const * p_runs, uint8_t count,
                                     uint32_t * p_now, char * p_out)
{
    s3_scan_cfg_t const cfg = scan_cfg();
    bool black = true;

    (void)s3_scan_edge(p_scan, &cfg, true, *p_now);
    for (uint8_t idx = 0U; idx < count; ++idx)
    {
        *p_now += p_runs[idx];
        black = !black;
        (void)s3_scan_edge(p_scan, &cfg, black, *p_now);
    }

    return scan_settle(p_scan, &cfg, 10000U, p_now, p_out);
}

/*
 * S3-U03: noise and broken frames (post-demo test, 2026-11-25). Over
 * every letter A..Z with a 2.5:1 print (100 ms narrow, 250 ms wide):
 *  - a 20 ms blip of the opposite level (one sample at the 20 ms
 *    sampling of F-01; sensor noise or a speck) inside any one of the
 *    29 runs should be decoded as the printed letter or rejected, never
 *    decoded as a different letter. KNOWN DEFECT F-18: a blip inside the
 *    payload makes 31 runs, the stray-mark path decodes a misaligned
 *    window, and 25 of the 234 payload cases give a wrong letter;
 *  - a frame whose start or stop character is a letter instead of the
 *    sentinel is rejected. P is left out: P is the mirror image of the
 *    sentinel, so P-x-P read backwards is exactly *-reverse(x)-*, which
 *    no two-way decoder can tell apart (P-Q-P reads as W);
 *  - the symbol framed by sentinels decodes, as a control.
 */
static void test_scan_noise(void)
{
    s3_scan_t scan;
    uint32_t now = 20000000U;
    bool b_never_wrong = true;
    unsigned int wrong = 0U;
    bool b_frames_rejected = true;
    bool b_controls = true;

    s3_scan_init(&scan);
    for (uint8_t idx = 0U; idx < S3_CODE39_SYMBOL_COUNT; ++idx)
    {
        uint16_t const pattern = g_s3_code39_alphabet[idx].pattern;
        char const letter = g_s3_code39_alphabet[idx].symbol;
        uint32_t runs[S3_BAR_FRAME_RUNS + 2U];
        char symbol = '\0';

        (void)make_runs(runs, S3_CODE39_SENTINEL, pattern,
                        S3_CODE39_SENTINEL, 100000U, 250000U);
        b_controls = b_controls
            && (S3_SCAN_DECODED == feed_runs_us(&scan, runs,
                                                S3_BAR_FRAME_RUNS, &now,
                                                &symbol))
            && (letter == symbol);

        for (uint8_t split = 0U; split < S3_BAR_FRAME_RUNS; ++split)
        {
            uint32_t noisy[S3_BAR_FRAME_RUNS + 2U];
            uint8_t count = 0U;

            (void)make_runs(runs, S3_CODE39_SENTINEL, pattern,
                            S3_CODE39_SENTINEL, 100000U, 250000U);
            for (uint8_t run = 0U; run < S3_BAR_FRAME_RUNS; ++run)
            {
                if (run == split)
                {
                    /* Half the run, a 20 ms blip, the rest. */
                    noisy[count] = runs[run] / 2U;
                    noisy[count + 1U] = 20000U;
                    noisy[count + 2U] = runs[run] - (runs[run] / 2U)
                                        - 20000U;
                    count = (uint8_t)(count + 3U);
                }
                else
                {
                    noisy[count] = runs[run];
                    ++count;
                }
            }
            symbol = '\0';
            if ((S3_SCAN_DECODED == feed_runs_us(&scan, noisy, count, &now,
                                                 &symbol))
                && (letter != symbol))
            {
                b_never_wrong = false;
                ++wrong;
            }
        }

        if ((idx > 0U) && ('P' != g_s3_code39_alphabet[idx - 1U].symbol))
        {
            /* The previous letter in place of the start, then the stop. */
            uint16_t const other = g_s3_code39_alphabet[idx - 1U].pattern;

            (void)make_runs(runs, other, pattern, other, 100000U, 250000U);
            b_frames_rejected = b_frames_rejected
                && (S3_SCAN_DECODED != feed_runs_us(&scan, runs,
                                                    S3_BAR_FRAME_RUNS, &now,
                                                    &symbol));
        }
    }
    check(b_controls, "framed letters decode (noise test control)");
    printf("noise blip: %u of %u payload cases gave a wrong letter\n",
           wrong, (unsigned int)(S3_CODE39_SYMBOL_COUNT * S3_BAR_ELEMENTS));
    known_defect(b_never_wrong, "F-18",
                 "a noise blip never gives a wrong letter");
    check(b_frames_rejected, "a letter in place of the sentinels is rejected");
}

/* Car captures of barcode F, 2026-11-17 (logs/2026-11-17_s3-bar-drive-4):
 * a stretched first bar and a 39..40 ms stray mark after the code. */
static void test_scan_recorded(void)
{
    static uint16_t const pass_517[31] =
    {
        540U, 99U, 440U, 200U, 599U, 220U, 159U, 500U,
        160U, 200U, 179U, 200U, 179U, 560U, 460U, 159U,
        459U, 180U, 119U, 200U, 160U, 240U, 439U, 179U,
        439U, 200U, 120U, 479U, 139U, 179U, 40U
    };
    static uint16_t const pass_642[31] =
    {
        500U, 100U, 419U, 179U, 520U, 239U, 160U, 559U,
        160U, 159U, 159U, 200U, 180U, 579U, 459U, 160U,
        460U, 180U, 139U, 200U, 180U, 220U, 420U, 199U,
        420U, 199U, 100U, 499U, 139U, 200U, 39U
    };
    uint16_t lead[31] = {0U};
    uint16_t two_marks[33] = {0U};
    s3_scan_t scan;
    uint32_t now = 1000000U;
    char symbol = '\0';

    s3_scan_init(&scan);
    check((S3_SCAN_DECODED == feed_runs_ms(&scan, pass_517, 31U, &now,
                                           &symbol)) && ('F' == symbol),
          "recorded pass 517 decodes F despite stray mark and first bar");
    symbol = '\0';
    check((S3_SCAN_DECODED == feed_runs_ms(&scan, pass_642, 31U, &now,
                                           &symbol)) && ('F' == symbol),
          "recorded pass 642 decodes F despite stray mark and first bar");

    /* The same code with its stray mark before it instead of after. */
    lead[0] = 40U;
    lead[1] = 200U;
    for (uint8_t idx = 0U; idx < 29U; ++idx)
    {
        lead[2U + idx] = pass_642[idx];
    }
    lead[2] = 160U;                     /* repaired first bar */
    symbol = '\0';
    check((S3_SCAN_DECODED == feed_runs_ms(&scan, lead, 31U, &now,
                                           &symbol)) && ('F' == symbol),
          "stray mark before the code is ignored");

    /* Two stray marks are more than the bounded allowance. */
    for (uint8_t idx = 0U; idx < 31U; ++idx)
    {
        two_marks[idx] = pass_642[idx];
    }
    two_marks[31] = 200U;
    two_marks[32] = 40U;
    symbol = 'x';
    check((S3_SCAN_ERROR == feed_runs_ms(&scan, two_marks, 33U, &now,
                                         &symbol)) && ('x' == symbol),
          "33 runs are rejected");

    /* Both sentinels damaged leaves nothing intact to rebuild from. */
    lead[2] = 500U;
    lead[28] = 500U;                    /* inside the closing sentinel */
    symbol = 'x';
    check((S3_SCAN_ERROR == feed_runs_ms(&scan, &lead[2], 29U, &now,
                                         &symbol)) && ('x' == symbol),
          "29 runs with both sentinels damaged are rejected");
}

static void test_scan_decode(void)
{
    s3_scan_t scan;
    uint32_t now = 5000000U;
    char symbol = '\0';

    s3_scan_init(&scan);
    for (uint8_t idx = 0U; idx < S3_CODE39_SYMBOL_COUNT; ++idx)
    {
        symbol = '\0';
        check((S3_SCAN_DECODED == feed_code(
                  &scan, g_s3_code39_alphabet[idx].pattern, 2000U, 5000U,
                  false, &now, &symbol))
              && (g_s3_code39_alphabet[idx].symbol == symbol),
              "decode each Code 39 letter forwards");

        symbol = '\0';
        check((S3_SCAN_DECODED == feed_code(
                  &scan, g_s3_code39_alphabet[idx].pattern, 2000U, 5000U,
                  true, &now, &symbol))
              && (g_s3_code39_alphabet[idx].symbol == symbol),
              "decode each Code 39 letter in reverse");
    }

    symbol = '\0';
    check((S3_SCAN_DECODED == feed_partial_z(&scan, false, 25U,
                                              &now, &symbol))
          && ('Z' == symbol),
          "decode 25-run Z with damaged trailing sentinel");

    symbol = '\0';
    check((S3_SCAN_DECODED == feed_partial_z(&scan, true, 25U,
                                              &now, &symbol))
          && ('Z' == symbol),
          "decode 25-run Z in reverse");

    symbol = '\0';
    check(S3_SCAN_ERROR == feed_partial_z(&scan, false, 23U,
                                           &now, &symbol),
          "reject a capture below the 24-run floor");

    symbol = '\0';
    check(S3_SCAN_ERROR == feed_code(&scan,
                                     S3_CODE39_PATTERN_Z ^ 0x001U,
                                     2000U, 5000U, false,
                                     &now, &symbol),
          "one-bit payload ambiguous between V and Z is rejected");

    symbol = '\0';
    check((S3_SCAN_DECODED == feed_code(&scan, S3_CODE39_PATTERN_F, 900U,
                                        2000U, false, &now, &symbol))
          && ('F' == symbol),
          "windows follow a faster pass");

    symbol = '\0';
    check((S3_SCAN_DECODED == feed_code(&scan, S3_CODE39_PATTERN_A, 160000U,
                                        500000U, false, &now, &symbol))
          && ('A' == symbol), "slow-pass widths fit framing");

    now = UINT32_MAX - 20000U;
    symbol = '\0';
    check((S3_SCAN_DECODED == feed_code(&scan, S3_CODE39_PATTERN_Z, 2000U,
                                        5000U, false, &now, &symbol))
          && ('Z' == symbol),
          "decode across the timer wrap");
}

static void test_scan_failures(void)
{
    s3_scan_cfg_t const cfg = scan_cfg();
    s3_scan_t scan;
    uint32_t now = 1000000U;
    char symbol = 'x';
    s3_scan_result_t result = S3_SCAN_NONE;

    s3_scan_init(&scan);
    check(S3_SCAN_ERROR == feed_code(&scan, S3_CODE39_PATTERN_A, 3000U, 3000U,
                                     false, &now, &symbol),
          "equal widths have no contrast");
    check(S3_SCAN_ERROR == feed_code(&scan, 0x111U, 2000U, 5000U, false,
                                     &now, &symbol),
          "unknown payload rejected");
    check(S3_SCAN_ERROR == feed_code(&scan, 0x188U, 2000U, 5000U, false,
                                     &now, &symbol),
          "two-error payload ambiguous between A and Z is rejected");
    check('x' == symbol, "failed decode leaves the symbol");

    (void)s3_scan_edge(&scan, &cfg, true, now);
    now += 20000U;
    (void)s3_scan_edge(&scan, &cfg, false, now);
    result = scan_settle(&scan, &cfg, 10000U, &now, &symbol);
    check(S3_SCAN_SINGLE_BAR == result, "one bar is a crossing line");

    (void)s3_scan_edge(&scan, &cfg, true, now);
    result = scan_settle(&scan, &cfg, 10000U, &now, &symbol);
    check(S3_SCAN_ERROR == result, "black longer than run_max aborts");

    (void)s3_scan_edge(&scan, &cfg, false, now);
    result = S3_SCAN_NONE;
    for (uint8_t idx = 0U; (idx < 70U) && (S3_SCAN_NONE == result); ++idx)
    {
        now += 1000U;
        result = s3_scan_edge(&scan, &cfg, (0U == (idx % 2U)), now);
    }
    check(S3_SCAN_ERROR == result, "too many runs overflow");

    s3_scan_init(&scan);
    (void)s3_scan_edge(&scan, &cfg, true, now);
    s3_scan_abort(&scan);
    (void)s3_scan_edge(&scan, &cfg, false, now + 1000U);
    symbol = '\0';
    now += 1000000U;
    check((S3_SCAN_DECODED == feed_code(&scan, S3_CODE39_PATTERN_A, 2000U,
                                        5000U, false, &now, &symbol))
          && ('A' == symbol),
          "decodes again after an abort");

    check(S3_SCAN_NONE == s3_scan_poll(NULL, &cfg, now, &symbol),
          "null scan poll");
    check(S3_SCAN_NONE == s3_scan_edge(&scan, NULL, true, now),
          "null scan config");
}

/* Sampled-sensor cases for s3_barcode. ADC levels follow the 2026-11-11
 * bench readings (white about 170, solid black about 1690, faded black
 * about 1650) and the run widths follow the 40 mm/s observation (narrow
 * 120..220 ms, wide 420..540 ms). They are synthetic, not robot data. */
#define TEST_SAMPLE_US      10000U
#define TEST_ADC_WHITE      170U
#define TEST_ADC_BLACK      1690U
#define TEST_ADC_FADED      1650U

typedef struct
{
    s3_barcode_t barcode;
    uint32_t     now_us;
    uint16_t     black_adc;
    uint8_t      decoded;
    uint8_t      errors;
    uint8_t      singles;
    char         last;
} sampled_t;

/* Hold one level for duration_us, sampling every 10 ms. */
static void sample_level(sampled_t * p_run, bool b_black,
                         uint32_t duration_us)
{
    uint16_t const adc = b_black ? p_run->black_adc : TEST_ADC_WHITE;

    for (uint32_t elapsed = 0U; elapsed < duration_us;
         elapsed += TEST_SAMPLE_US)
    {
        char symbol = '\0';
        s3_scan_result_t const result = s3_barcode_update(
            &p_run->barcode, adc, p_run->now_us, &symbol);

        if (S3_SCAN_DECODED == result)
        {
            ++p_run->decoded;
            p_run->last = symbol;
        }
        else if (S3_SCAN_ERROR == result)
        {
            ++p_run->errors;
        }
        else if (S3_SCAN_SINGLE_BAR == result)
        {
            ++p_run->singles;
        }
        else
        {
            /* Nothing finished in this period. */
        }
        p_run->now_us += TEST_SAMPLE_US;
    }
}

/* Pass one printed code under the sensor, then a quiet white zone. */
static void sample_code(sampled_t * p_run, uint16_t symbol,
                        uint32_t narrow_us, uint32_t wide_us, bool b_reverse)
{
    uint16_t const order[3] =
    {
        S3_CODE39_SENTINEL, symbol, S3_CODE39_SENTINEL
    };

    for (uint8_t ch = 0U; ch < 3U; ++ch)
    {
        uint16_t const pattern = order[b_reverse ? (2U - ch) : ch];

        for (uint8_t idx = 0U; idx < S3_BAR_ELEMENTS; ++idx)
        {
            uint8_t const bit = b_reverse ? idx : (uint8_t)(8U - idx);
            bool const b_wide = (0U != (pattern & (1U << bit)));

            sample_level(p_run, (0U == (idx % 2U)),
                         b_wide ? wide_us : narrow_us);
        }
        if (ch < 2U)
        {
            sample_level(p_run, false, narrow_us);  /* inter-char gap */
        }
    }
    sample_level(p_run, false, 1000000U);           /* quiet zone */
}

static void sampled_init(sampled_t * p_run, uint32_t now_us)
{
    sampled_t const zero = {0};

    *p_run = zero;
    p_run->now_us = now_us;
    p_run->black_adc = TEST_ADC_BLACK;
    check(s3_barcode_init(&p_run->barcode), "barcode init");
    sample_level(p_run, false, 200000U);            /* settle on white */
}

static void test_sampled_barcode(void)
{
    sampled_t run;
    bool b_all_forward = true;
    bool b_all_reverse = true;

    sampled_init(&run, 1000000U);
    for (uint8_t idx = 0U; idx < S3_CODE39_SYMBOL_COUNT; ++idx)
    {
        uint8_t const before = run.decoded;

        sample_code(&run, g_s3_code39_alphabet[idx].pattern,
                    150000U, 450000U, false);
        b_all_forward = b_all_forward && (before + 1U == run.decoded)
                        && (g_s3_code39_alphabet[idx].symbol == run.last);

        sample_code(&run, g_s3_code39_alphabet[idx].pattern,
                    150000U, 450000U, true);
        b_all_reverse = b_all_reverse && ((before + 2U) == run.decoded)
                        && (g_s3_code39_alphabet[idx].symbol == run.last);
    }
    check(b_all_forward, "sampled A-Z decodes forwards at 10 ms");
    check(b_all_reverse, "sampled A-Z decodes in reverse at 10 ms");
    check((0U == run.errors) && (0U == run.singles),
          "sampled alphabet has no errors");

    sampled_init(&run, UINT32_MAX - 3000000U);
    sample_code(&run, S3_CODE39_PATTERN_B, 120000U, 540000U, false);
    check((1U == run.decoded) && ('B' == run.last),
          "extreme observed widths decode across the timer wrap");

    /* Pass of 2026-11-17 (capture 7443): start sentinel, gaps and F were
     * read, then the car paused on a wide bar of the closing sentinel. A
     * 1.5 s bar no longer aborts the capture; the rebuild decodes F. */
    {
        static uint16_t const pass_7443[29] =
        {
            160U, 459U, 180U, 100U, 560U, 80U, 479U, 160U, 139U,
            139U,
            159U, 160U, 459U, 119U, 480U, 419U, 219U, 79U, 200U,
            119U,
            179U, 480U, 199U, 80U, 1500U, 160U, 450U, 160U, 160U
        };

        sampled_init(&run, 1000000U);
        for (uint8_t idx = 0U; idx < 29U; ++idx)
        {
            sample_level(&run, (0U == (idx % 2U)),
                         (uint32_t)pass_7443[idx] * 1000U);
        }
        sample_level(&run, false, 1000000U);
        check((1U == run.decoded) && ('F' == run.last) && (0U == run.errors),
              "recorded pass with a 1.5 s pause on the closing bar is F");

        /* A bar held past the 2.5 s limit still aborts. */
        sampled_init(&run, 1000000U);
        sample_level(&run, true, 150000U);
        sample_level(&run, false, 150000U);
        sample_level(&run, true, 2600000U);
        check((1U == run.errors) && (0U == run.decoded),
              "a bar held over 2.5 s aborts the capture");
        sample_level(&run, false, 1000000U);
    }

    sampled_init(&run, 1000000U);
    run.black_adc = TEST_ADC_FADED;
    sample_code(&run, S3_CODE39_PATTERN_C, 150000U, 450000U, false);
    check((1U == run.decoded) && ('C' == run.last), "faded black decodes");

    sampled_init(&run, 1000000U);
    run.black_adc = 500U;       /* below baseline + 400: never black */
    sample_code(&run, S3_CODE39_PATTERN_D, 150000U, 450000U, false);
    check((0U == run.decoded) && (0U == run.errors),
          "weak contrast is ignored, not decoded");

    sampled_init(&run, 1000000U);
    sample_level(&run, true, 100000U);
    sample_level(&run, false, 1000000U);
    check((1U == run.singles) && (0U == run.decoded),
          "one tape crossing is a single bar");

    sampled_init(&run, 1000000U);
    sample_level(&run, true, 150000U);
    sample_level(&run, false, 150000U);
    s3_barcode_abort(&run.barcode);
    sample_level(&run, true, 150000U);  /* mid-code: not a clean start */
    sample_level(&run, false, 1000000U);
    check((0U == run.decoded) && (0U == run.errors) && (1U == run.singles),
          "abort drops the capture; next bar starts a new one");

    sampled_init(&run, 1000000U);
    check(!s3_barcode_active(&run.barcode), "idle on white is not active");
    sample_level(&run, true, 150000U);
    sample_level(&run, false, 150000U);
    check(!s3_barcode_active(&run.barcode), "one bar is not yet a barcode");
    sample_level(&run, true, 150000U);
    sample_level(&run, false, 10000U);
    check(s3_barcode_active(&run.barcode), "three plausible runs: active");
    sample_level(&run, false, 140000U);
    sample_level(&run, true, 50000U);
    sample_level(&run, false, 10000U);
    check(s3_barcode_active(&run.barcode),
          "a later short run keeps it active");
    sample_level(&run, false, 1000000U);
    check(!s3_barcode_active(&run.barcode), "quiet white ends it");
    sample_level(&run, true, 150000U);
    sample_level(&run, false, 150000U);
    sample_level(&run, true, 150000U);
    sample_level(&run, false, 10000U);
    s3_barcode_abort(&run.barcode);
    check(!s3_barcode_active(&run.barcode), "abort ends it");
    check(!s3_barcode_active(NULL), "null barcode is not active");

    /* Blips like those logged on 2026-11-17 (19..40 ms) never qualify. */
    sampled_init(&run, 1000000U);
    for (uint8_t idx = 0U; idx < 6U; ++idx)
    {
        sample_level(&run, true, 20000U);
        sample_level(&run, false, 40000U);
    }
    check(!s3_barcode_active(&run.barcode), "edge blips are not a barcode");
    sample_level(&run, false, 1000000U);

    /* A crossing line is one bar, so it never softens the steering. */
    sample_level(&run, true, 300000U);
    check(!s3_barcode_active(&run.barcode), "crossing line is not a barcode");
    sample_level(&run, false, 1000000U);

    check(!s3_barcode_init(NULL), "null barcode init");
    check(S3_SCAN_NONE == s3_barcode_update(NULL, 0U, 0U, &run.last),
          "null barcode update");
    check(S3_SCAN_NONE == s3_barcode_update(&run.barcode, 0U, 0U, NULL),
          "null barcode symbol");
}

static void test_unfiltered(void)
{
    s3_ir_adaptive_t filtered = {0};
    s3_ir_adaptive_t raw = {0};
    s3_ir_adaptive_config_t const config =
    {
        169U, 154U, 184U, 400U, 300U
    };

    check(s3_ir_adaptive_init(&filtered, &config)
          && s3_ir_adaptive_init(&raw, &config), "unfiltered init");
    check(!s3_ir_adaptive_update(&filtered, 700U),
          "filtered first step stays white");
    check(s3_ir_adaptive_update_raw(&raw, 700U),
          "unfiltered crosses the threshold at once");
    check(700U == raw.filtered, "unfiltered keeps the sample exactly");
    check(s3_ir_adaptive_update_raw(&raw, 500U),
          "unfiltered hysteresis holds black above release");
    check(!s3_ir_adaptive_update_raw(&raw, 400U),
          "unfiltered releases at the white margin");
    check(!s3_ir_adaptive_update_raw(&raw, 4096U),
          "unfiltered rejects out-of-range ADC");
}

int main(void)
{
    test_ir();
    test_unfiltered();
    test_barcode();
    test_navigation();
    test_pipeline();
    test_scan_decode();
    test_scan_failures();
    test_scan_recorded();
    test_sampled_barcode();
    test_scan_robustness();
    test_scan_noise();
    printf("%u checks, %u failures, %u known defects\n", g_checks, g_failures,
           g_known);
    return (0U == g_failures) ? 0 : 1;
}

/** End of file. */

