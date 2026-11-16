/** @file test_track_scan.c Synthetic cases for s3_track and s3_scan.
 *
 * Inputs are invented timings and sensor masks, not robot measurements.
 * Barcode cases use the standard Code 39 A-to-Z lookup table. Hardware
 * repeatability still has to be checked.
 */
#include "s3_scan.h"
#include "s3_config.h"
#include "s3_track.h"
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

    out = track_run(&track, &cfg, 0U, 200U, &now, &events);
    check((!out.lost) && out.valid && (S3_LINE_STRAIGHT == out.line),
          "centred straddle does not become lost");

    /* Exercise the generic loss timer with a geometry in which both-white
     * is not a valid centred decision. */
    cfg.decisions[0U] = S3_LINE_UNKNOWN;
    events = 0U;
    check(s3_track_init(&track, &cfg, now), "non-straddle config valid");

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
    s3_scan_cfg_t cfg = {S3_BAR_RUN_MAX_US, S3_BAR_QUIET_US,
                         S3_BAR_SENTINEL, g_s3_code39_alphabet,
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
    uint16_t const order[3] =
    {
        S3_BAR_SENTINEL, symbol, S3_BAR_SENTINEL
    };
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

    for (uint8_t tick = 0U; (tick < 100U) && (S3_SCAN_NONE == result); ++tick)
    {
        *p_now += 10000U;
        result = s3_scan_poll(p_scan, &cfg, *p_now, p_out);
    }

    return result;
}

/* Replay the observed 25-run smaller-Z failure. Its first 20 runs contain
 * sentinel, gap, payload and gap; only the terminal sentinel is damaged. */
static s3_scan_result_t feed_partial_z(s3_scan_t * p_scan, bool reverse,
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
    s3_scan_cfg_t const cfg = scan_cfg();
    bool black = true;
    s3_scan_result_t result = S3_SCAN_NONE;

    (void)s3_scan_edge(p_scan, &cfg, true, *p_now);
    for (uint8_t idx = 0U; idx < run_count; ++idx)
    {
        uint8_t const source = reverse
            ? (uint8_t)((sizeof(runs) / sizeof(runs[0])) - 1U - idx)
            : idx;

        *p_now += runs[source];
        black = !black;
        (void)s3_scan_edge(p_scan, &cfg, black, *p_now);
    }

    for (uint8_t tick = 0U; (tick < 100U) && (S3_SCAN_NONE == result); ++tick)
    {
        *p_now += 10000U;
        result = s3_scan_poll(p_scan, &cfg, *p_now, p_out);
    }

    return result;
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
          "decode observed 25-run smaller Z with damaged trailing sentinel");

    symbol = '\0';
    check((S3_SCAN_DECODED == feed_partial_z(&scan, true, 25U,
                                              &now, &symbol))
          && ('Z' == symbol),
          "decode observed 25-run smaller Z in reverse");

    symbol = '\0';
    check(S3_SCAN_ERROR == feed_partial_z(&scan, false, 23U,
                                           &now, &symbol),
          "reject a capture below the 80 percent floor");

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
          && ('A' == symbol), "observed 40 mm/s widths fit framing");

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
    result = S3_SCAN_NONE;
    for (uint8_t tick = 0U; (tick < 100U) && (S3_SCAN_NONE == result); ++tick)
    {
        now += 10000U;
        result = s3_scan_poll(&scan, &cfg, now, &symbol);
    }
    check(S3_SCAN_SINGLE_BAR == result, "one bar is a crossing line");

    (void)s3_scan_edge(&scan, &cfg, true, now);
    result = S3_SCAN_NONE;
    for (uint8_t tick = 0U; (tick < 100U) && (S3_SCAN_NONE == result); ++tick)
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
    check((S3_SCAN_DECODED == feed_code(&scan, S3_CODE39_PATTERN_A, 2000U,
                                        5000U, false, &now, &symbol))
          && ('A' == symbol),
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
