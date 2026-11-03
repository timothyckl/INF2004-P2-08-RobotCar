/** @file test_track_scan.c Synthetic cases for s3_track and s3_scan.
 *
 * Inputs are invented timings and sensor masks, not robot measurements.
 * Barcode cases use the standard Code 39 patterns for '*' and A..D, which
 * the task wires in; the printed course barcodes still have to be checked.
 */
#include "s3_scan.h"
#include "s3_track.h"
#include <stddef.h>
#include <stdio.h>

#define T_SENTINEL  0x094U  /* Code 39 '*' */
#define T_SYM_A     0x109U
#define T_SYM_B     0x049U
#define T_SYM_C     0x148U
#define T_SYM_D     0x019U

static unsigned int g_checks = 0U;
static unsigned int g_failures = 0U;

static s3_symbol_t const g_alphabet[4] =
{
    {T_SYM_A, 'A'}, {T_SYM_B, 'B'}, {T_SYM_C, 'C'}, {T_SYM_D, 'D'}
};

static void check(bool passed, char const * p_name)
{
    ++g_checks;
    if (!passed)
    {
        ++g_failures;
        printf("FAIL: %s\n", p_name);
    }
}

/* Feed the same mask n times, 10 ms apart; return the last output. */
static s3_track_out_t track_run(s3_track_t * p_track,
                                s3_track_cfg_t const * p_cfg, uint8_t mask,
                                uint32_t n, uint32_t * p_now,
                                uint8_t * p_events)
{
    s3_track_out_t out = {S3_LINE_UNKNOWN, 0, 0U, 0U, false, false, false};

    for (uint32_t idx = 0U; idx < n; ++idx)
    {
        *p_now += 10U;
        s3_track_update(p_track, p_cfg, mask, *p_now, &out);
        if (NULL != p_events)
        {
            *p_events = (uint8_t)(*p_events | out.events);
        }
    }

    return out;
}

static void test_track_table(void)
{
    s3_track_cfg_t cfg;
    s3_track_t track;
    uint32_t now = 1000U;
    uint8_t events = 0U;
    s3_track_out_t out;

    s3_track_default_cfg(&cfg);
    check(s3_track_init(&track, &cfg, now), "default config valid");

    out = track_run(&track, &cfg, 0U, 1U, &now, NULL);
    check((S3_LINE_UNKNOWN == out.line) && (!out.valid)
          && (0 == out.steer_permille), "unsettled sample gives no steer");
    out = track_run(&track, &cfg, 0U, 1U, &now, NULL);
    check((S3_LINE_STRAIGHT == out.line) && out.valid
          && (0 == out.steer_permille), "both white straddle is straight");

    out = track_run(&track, &cfg, S3_TRACK_LEFT_BIT, 2U, &now, NULL);
    check((S3_LINE_LEFT == out.line) && (500 == out.steer_permille)
          && (S3_TRACK_SIDE_LEFT == out.last_side),
          "line under front-left steers left");
    out = track_run(&track, &cfg, S3_TRACK_RIGHT_BIT, 2U, &now, NULL);
    check((S3_LINE_RIGHT == out.line) && (-500 == out.steer_permille)
          && (S3_TRACK_SIDE_RIGHT == out.last_side),
          "line under front-right steers right");

    out = track_run(&track, &cfg, S3_TRACK_FRONT_BITS, 5U, &now, &events);
    check((S3_LINE_JUNCTION == out.line) && (0 == out.steer_permille)
          && (S3_TRACK_EV_JUNCTION == (events & S3_TRACK_EV_JUNCTION))
          && (S3_TRACK_SIDE_RIGHT == out.last_side),
          "both black is a junction, side unchanged");
    events = 0U;
    out = track_run(&track, &cfg, S3_TRACK_FRONT_BITS, 5U, &now, &events);
    check(0U == (events & S3_TRACK_EV_JUNCTION), "junction raised once");

    /* The barcode bit (bit 2) never changes the line decision. */
    out = track_run(&track, &cfg, 0x04U, 3U, &now, NULL);
    check(S3_LINE_STRAIGHT == out.line, "barcode bit alone is ignored");
    out = track_run(&track, &cfg, 0x04U | S3_TRACK_LEFT_BIT, 3U, &now, NULL);
    check(S3_LINE_LEFT == out.line, "barcode bit does not mask left");
}

static void test_track_lost_fault(void)
{
    s3_track_cfg_t cfg;
    s3_track_t track;
    uint32_t now = UINT32_MAX - 500U;   /* loss timing across the wrap */
    uint8_t events = 0U;
    s3_track_out_t out;

    s3_track_default_cfg(&cfg);
    check(s3_track_init(&track, &cfg, now), "init near timer wrap");

    out = track_run(&track, &cfg, 0U, 149U, &now, &events);
    check((!out.lost) && (0U == events), "not lost before lost_ms");
    out = track_run(&track, &cfg, 0U, 1U, &now, &events);
    check(out.lost && (!out.valid) && (S3_TRACK_EV_LOST == events),
          "lost at lost_ms, once");
    events = 0U;
    out = track_run(&track, &cfg, 0U, 50U, &now, &events);
    check(out.lost && (0U == events), "lost stays without new events");
    out = track_run(&track, &cfg, S3_TRACK_RIGHT_BIT, 1U, &now, &events);
    check((!out.lost) && (S3_TRACK_EV_FOUND == (events & S3_TRACK_EV_FOUND)),
          "black after loss raises found");

    events = 0U;
    /* The first both-black sample starts the timer: sample 201 is 2000 ms
     * after it. */
    out = track_run(&track, &cfg, S3_TRACK_FRONT_BITS, 200U, &now, &events);
    check(!out.fault, "both black shorter than fault_ms is no fault");
    out = track_run(&track, &cfg, S3_TRACK_FRONT_BITS, 1U, &now, &events);
    check(out.fault && (!out.valid) && (0 == out.steer_permille)
          && (S3_TRACK_EV_FAULT == (events & S3_TRACK_EV_FAULT)),
          "both black for fault_ms faults");
    out = track_run(&track, &cfg, 0U, 5U, &now, NULL);
    check(out.fault && (!out.valid), "fault stays latched");
    check(s3_track_init(&track, &cfg, now) && (!track.fault),
          "init clears the fault");
}

static void test_track_config(void)
{
    s3_track_cfg_t cfg;
    s3_track_t track;

    s3_track_default_cfg(&cfg);
    cfg.decisions[4U] = S3_LINE_STRAIGHT;
    check(!s3_track_init(&track, &cfg, 0U), "barcode bit decision refused");
    s3_track_default_cfg(&cfg);
    cfg.filter_samples = 0U;
    check(!s3_track_init(&track, &cfg, 0U), "zero filter refused");
    s3_track_default_cfg(&cfg);
    cfg.steer_side_permille = 0;
    check(!s3_track_init(&track, &cfg, 0U), "zero steer refused");
    cfg.steer_side_permille = S3_TRACK_STEER_MAX + 1;
    check(!s3_track_init(&track, &cfg, 0U), "over-range steer refused");
    s3_track_default_cfg(&cfg);
    cfg.lost_ms = 0U;
    check(!s3_track_init(&track, &cfg, 0U), "zero lost_ms refused");
    check(!s3_track_init(NULL, &cfg, 0U), "null track refused");
}

static s3_scan_cfg_t scan_cfg(void)
{
    s3_scan_cfg_t cfg = {400000U, 250000U, T_SENTINEL, g_alphabet, 4U};

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
        bool const is_wide = (0U != (pattern & (1U << (8U - idx))));

        *p_now += is_wide ? wide : narrow;
        *p_black = !*p_black;
        (void)s3_scan_edge(p_scan, p_cfg, *p_black, *p_now);
    }
}

/* Drive one framed code past the sensor; reverse feeds it backwards. */
static s3_scan_result_t feed_code(s3_scan_t * p_scan, uint16_t symbol,
                                  uint32_t narrow, uint32_t wide,
                                  bool reverse, uint32_t * p_now,
                                  char * p_out)
{
    s3_scan_cfg_t const cfg = scan_cfg();
    uint16_t const order[3] = {T_SENTINEL, symbol, T_SENTINEL};
    bool black = true;
    s3_scan_result_t result = S3_SCAN_NONE;

    (void)s3_scan_edge(p_scan, &cfg, true, *p_now);     /* first bar */

    for (uint8_t c = 0U; c < 3U; ++c)
    {
        uint16_t pattern = order[c];

        if (reverse)
        {
            uint16_t flipped = 0U;

            pattern = order[2U - c];
            for (uint8_t bit = 0U; bit < S3_BAR_ELEMENTS; ++bit)
            {
                flipped = (uint16_t)(flipped << 1U);
                flipped = (uint16_t)(flipped | ((pattern >> bit) & 1U));
            }
            pattern = flipped;
        }

        /* Each character starts on a bar (black); feed_char sends the edge
         * that ends each element. */
        feed_char(p_scan, &cfg, pattern, narrow, wide, &black, p_now);

        if (c < 2U)
        {
            *p_now += narrow;               /* inter-character gap */
            black = true;
            (void)s3_scan_edge(p_scan, &cfg, true, *p_now);
        }
    }

    for (uint8_t tick = 0U; (tick < 40U) && (S3_SCAN_NONE == result); ++tick)
    {
        *p_now += 10000U;
        result = s3_scan_poll(p_scan, &cfg, *p_now, p_out);
    }

    return result;
}

static void test_scan_decode(void)
{
    uint16_t const symbols[4] = {T_SYM_A, T_SYM_B, T_SYM_C, T_SYM_D};
    char const expected[4] = {'A', 'B', 'C', 'D'};
    s3_scan_t scan;
    uint32_t now = 5000000U;
    char symbol = '\0';

    s3_scan_init(&scan);
    for (uint8_t idx = 0U; idx < 4U; ++idx)
    {
        symbol = '\0';
        check((S3_SCAN_DECODED == feed_code(&scan, symbols[idx], 2000U, 5000U,
                                            false, &now, &symbol))
              && (expected[idx] == symbol), "decode A..D forwards");
    }

    symbol = '\0';
    check((S3_SCAN_DECODED == feed_code(&scan, T_SYM_B, 2000U, 5000U, true,
                                        &now, &symbol)) && ('B' == symbol),
          "decode a code read in reverse");

    symbol = '\0';
    check((S3_SCAN_DECODED == feed_code(&scan, T_SYM_C, 900U, 2000U, false,
                                        &now, &symbol)) && ('C' == symbol),
          "windows follow a faster pass");

    now = UINT32_MAX - 20000U;
    symbol = '\0';
    check((S3_SCAN_DECODED == feed_code(&scan, T_SYM_D, 2000U, 5000U, false,
                                        &now, &symbol)) && ('D' == symbol),
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
    check(S3_SCAN_ERROR == feed_code(&scan, T_SYM_A, 3000U, 3000U, false,
                                     &now, &symbol),
          "equal widths have no contrast");
    check(S3_SCAN_ERROR == feed_code(&scan, 0x111U, 2000U, 5000U, false,
                                     &now, &symbol),
          "unknown payload rejected");
    check('x' == symbol, "failed decode leaves the symbol");

    (void)s3_scan_edge(&scan, &cfg, true, now);
    now += 20000U;
    (void)s3_scan_edge(&scan, &cfg, false, now);
    result = S3_SCAN_NONE;
    for (uint8_t tick = 0U; (tick < 40U) && (S3_SCAN_NONE == result); ++tick)
    {
        now += 10000U;
        result = s3_scan_poll(&scan, &cfg, now, &symbol);
    }
    check(S3_SCAN_SINGLE_BAR == result, "one bar is a crossing line");

    (void)s3_scan_edge(&scan, &cfg, true, now);
    result = S3_SCAN_NONE;
    for (uint8_t tick = 0U; (tick < 50U) && (S3_SCAN_NONE == result); ++tick)
    {
        now += 10000U;
        result = s3_scan_poll(&scan, &cfg, now, &symbol);
    }
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
    check((S3_SCAN_DECODED == feed_code(&scan, T_SYM_A, 2000U, 5000U, false,
                                        &now, &symbol)) && ('A' == symbol),
          "decodes again after an abort");
}

int main(void)
{
    test_track_table();
    test_track_lost_fault();
    test_track_config();
    test_scan_decode();
    test_scan_failures();
    printf("%u checks, %u failures\n", g_checks, g_failures);
    return (0U == g_failures) ? 0 : 1;
}

/** End of file. */
