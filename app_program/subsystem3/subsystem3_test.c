/**
 * @file subsystem3_test.c
 * @brief On-target tests for subsystem 3. Built only with: make TEST=1
 *
 * Record unit results in testing/subsystem3/TEST_LOG.md and the INT-
 * result in testing/integration/TEST_LOG.md. The logic cases use
 * synthetic samples. They do not validate real sensors or bar widths on
 * the track.
 */
#include "subsystem3_test.h"

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST

/* Synthetic Code 39 widths. The patterns are written out again here from
   the Code 39 table (ISO/IEC 16388), so the test does not reuse the
   decoder's own table. */
#define S3_TEST_NARROW          (2u)
#define S3_TEST_WIDE            (5u)
#define S3_TEST_STAR            (0x094u)
#define S3_TEST_COMMANDS        (4u)
#define S3_TEST_CODE_CHARS      (3u)
#define S3_TEST_CHAR_ELEMENTS   (9u)
#define S3_TEST_LAST_ELEMENT    (S3_TEST_CHAR_ELEMENTS - 1u)

#define S3_TEST_PATTERNS        (6u)    /* Valid single-line patterns.   */
#define S3_TEST_QUIET_SAMPLES   (20u)   /* More than a quiet zone needs. */
#define S3_TEST_OVERFLOW_RUNS   (40u)   /* More than a barcode holds.    */
#define S3_TEST_RUN_WIDTH       (2u)
#define S3_TEST_LONG_WHITE      (30u)
#define S3_TEST_BAR_WIDTH       (3u)
#define S3_TEST_CORRUPT_INDEX   (10u)   /* First wide bar of 'A'.        */
#define S3_TEST_GAP_INDEX       (9u)    /* First inter-character gap.    */
#define S3_TEST_UNKNOWN_CHAR    (0x1C0u)

#define S3_TEST_SAMPLE_WAIT_MS  (100)
#define S3_TEST_MIN_SAMPLES     (5u)    /* About 10 are expected.        */

static uint16_t const g_test_patterns[S3_TEST_COMMANDS] =
{
    0x109u, 0x049u, 0x148u, 0x019u          /* A, B, C, D */
};

static barcode_cmd_t const g_test_cmds[S3_TEST_COMMANDS] =
{
    BARCODE_CMD_LEFT, BARCODE_CMD_RIGHT, BARCODE_CMD_STRAIGHT,
    BARCODE_CMD_UTURN
};

/** Read the starter example's statistics under its mutex. */
static APP_STATS s3_test_read_stats (void);

/** Build the widths of "*X*" for the middle character pattern. */
static void s3_test_build (uint16_t middle, uint16_t p_widths[]);

/** S3-U02: each single-line pattern gives its position. */
static bool s3_test_positions (void);

/** S3-U03: LOST only after the debounce count. */
static bool s3_test_lost (void);

/** S3-U04: FOUND only after the debounce count. */
static bool s3_test_found (void);

/** S3-U05: a persistent invalid pattern gives a sticky fault. */
static bool s3_test_fault (void);

/** S3-U06 and S3-U07: decode A to D forwards or in reverse. */
static bool s3_test_decode (bool b_reverse);

/** S3-U08: reject corrupt, unknown and malformed input. */
static bool s3_test_reject (void);

/** S3-U09: capture decodes, detects one bar and reports overflow. */
static bool s3_test_capture (void);

/** S3-U11: the line sensor takes samples. */
static bool s3_test_sampling (void);

void
subsystem3_test (void)
{
    TEST_INIT();

    /* Starter example. INT-01 (IF-01): at one sample per 300 ms, 2 s
       should bring at least 5. */
    APP_STATS const before = s3_test_read_stats();

    tk_dly_tsk(2000);

    APP_STATS const after = s3_test_read_stats();

    TEST_CHECK("INT-01", after.received >= (before.received + 5u),
               "samples flow from subsystem 2 to subsystem 3");
    TEST_CHECK("S3-U01",
               (after.received > 0u) && (after.min <= after.max)
               && (after.min >= 0) && (after.max <= 100),
               "min <= max, both within 0..100");

    /* Line and barcode logic. */
    TEST_CHECK("S3-U02", s3_test_positions(),
               "each single-line pattern gives its position");
    TEST_CHECK("S3-U03", s3_test_lost(),
               "line lost only after N empty samples, side kept");
    TEST_CHECK("S3-U04", s3_test_found(),
               "line reacquired after N line samples");
    TEST_CHECK("S3-U05", s3_test_fault(),
               "persistent invalid pattern gives sticky fault");
    TEST_CHECK("S3-U06", s3_test_decode(false),
               "Code 39 A-D decode to their commands");
    TEST_CHECK("S3-U07", s3_test_decode(true),
               "barcodes read backwards decode the same");
    TEST_CHECK("S3-U08", s3_test_reject(),
               "corrupt, unknown and malformed codes rejected");
    TEST_CHECK("S3-U09", s3_test_capture(),
               "capture: decode, single bar and overflow");
    TEST_CHECK("S3-U11", s3_test_sampling(),
               "line sensor samples every period");

    TEST_SUMMARY("subsystem3");
}

static APP_STATS
s3_test_read_stats (void)
{
    APP_STATS stats;

    (void) tk_loc_mtx(mtx_stats, TMO_FEVR);
    stats = app_stats;
    (void) tk_unl_mtx(mtx_stats);

    return (stats);
}

static void
s3_test_build (uint16_t middle, uint16_t p_widths[])
{
    uint16_t const chars[S3_TEST_CODE_CHARS] =
    {
        S3_TEST_STAR, middle, S3_TEST_STAR
    };
    uint32_t       slot = 0u;

    for (uint32_t char_index = 0u; char_index < S3_TEST_CODE_CHARS;
         char_index++)
    {
        for (uint32_t element = 0u; element < S3_TEST_CHAR_ELEMENTS;
             element++)
        {
            uint32_t const mask = 1u << (S3_TEST_LAST_ELEMENT - element);

            p_widths[slot] = (0u != (chars[char_index] & mask))
                             ? S3_TEST_WIDE : S3_TEST_NARROW;
            slot++;
        }

        if (char_index < (S3_TEST_CODE_CHARS - 1u))
        {
            p_widths[slot] = S3_TEST_NARROW;    /* Inter-character gap. */
            slot++;
        }
    }
}

static bool
s3_test_positions (void)
{
    uint8_t const bits[S3_TEST_PATTERNS]     =
    {
        0x2u, 0x6u, 0x4u, 0x3u, 0x1u, 0x7u
    };
    int8_t const  expected[S3_TEST_PATTERNS] = {0, -1, -2, 1, 2, 0};
    line_track_t  track;
    bool          b_passed = true;

    line_track_init(&track);

    for (uint32_t index = 0u; index < S3_TEST_PATTERNS; index++)
    {
        (void) line_track_update(&track, bits[index]);
        b_passed = b_passed && (track.b_valid)
                   && (expected[index] == track.position);
    }

    return (b_passed);
}

static bool
s3_test_lost (void)
{
    line_track_t track;
    bool         b_quiet = true;

    line_track_init(&track);
    (void) line_track_update(&track, 0x6u);     /* Line slightly left. */

    for (uint32_t index = 1u; index < LINE_TRACK_LOST_SAMPLES; index++)
    {
        b_quiet = b_quiet && (0u == line_track_update(&track, 0u));
    }

    uint8_t const events = line_track_update(&track, 0u);

    return (b_quiet && (LINE_TRACK_EVENT_LOST == events)
            && (LINE_TRACK_LOST == track.mode) && (!track.b_valid)
            && (track.position < 0));
}

static bool
s3_test_found (void)
{
    line_track_t track;
    bool         b_quiet = true;

    line_track_init(&track);

    for (uint32_t index = 0u; index < LINE_TRACK_LOST_SAMPLES; index++)
    {
        (void) line_track_update(&track, 0u);
    }

    for (uint32_t index = 1u; index < LINE_TRACK_FOUND_SAMPLES; index++)
    {
        b_quiet = b_quiet && (0u == line_track_update(&track, 0x2u));
    }

    uint8_t const events = line_track_update(&track, 0x2u);

    return (b_quiet && (LINE_TRACK_EVENT_FOUND == events)
            && (LINE_TRACK_FOLLOWING == track.mode));
}

static bool
s3_test_fault (void)
{
    line_track_t track;
    uint8_t      events = 0u;

    line_track_init(&track);

    for (uint32_t index = 0u; index < LINE_TRACK_FAULT_SAMPLES; index++)
    {
        events |= line_track_update(&track, 0x5u);
    }

    /* A good sample afterwards must not clear the fault. */
    return ((LINE_TRACK_EVENT_FAULT == events)
            && (0u == line_track_update(&track, 0x2u))
            && (LINE_TRACK_FAULT == track.mode) && (!track.b_valid));
}

static bool
s3_test_decode (bool b_reverse)
{
    uint16_t widths[BARCODE_MAX_ELEMENTS];
    uint16_t input[BARCODE_MAX_ELEMENTS];
    bool     b_passed = true;

    for (uint32_t command = 0u; command < S3_TEST_COMMANDS; command++)
    {
        barcode_cmd_t cmd = BARCODE_CMD_NONE;

        s3_test_build(g_test_patterns[command], widths);

        for (uint32_t index = 0u; index < BARCODE_MAX_ELEMENTS; index++)
        {
            input[index] = b_reverse
                           ? widths[BARCODE_MAX_ELEMENTS - 1u - index]
                           : widths[index];
        }

        b_passed = b_passed
                   && (barcode_decode(input, BARCODE_MAX_ELEMENTS, &cmd))
                   && (g_test_cmds[command] == cmd);
    }

    return (b_passed);
}

static bool
s3_test_reject (void)
{
    uint16_t      widths[BARCODE_MAX_ELEMENTS];
    barcode_cmd_t cmd      = BARCODE_CMD_NONE;
    bool          b_passed = true;

    s3_test_build(g_test_patterns[0u], widths);
    widths[S3_TEST_CORRUPT_INDEX] = S3_TEST_NARROW;
    b_passed = b_passed
               && (!barcode_decode(widths, BARCODE_MAX_ELEMENTS, &cmd));

    s3_test_build(g_test_patterns[0u], widths);
    widths[S3_TEST_GAP_INDEX] = S3_TEST_WIDE;
    b_passed = b_passed
               && (!barcode_decode(widths, BARCODE_MAX_ELEMENTS, &cmd));

    s3_test_build(S3_TEST_UNKNOWN_CHAR, widths);
    b_passed = b_passed
               && (!barcode_decode(widths, BARCODE_MAX_ELEMENTS, &cmd));

    b_passed = b_passed
               && (!barcode_decode(widths, BARCODE_MAX_ELEMENTS - 1u, &cmd))
               && (!barcode_decode(NULL, BARCODE_MAX_ELEMENTS, &cmd))
               && (BARCODE_CMD_NONE == cmd);

    return (b_passed);
}

static bool
s3_test_capture (void)
{
    uint16_t         widths[BARCODE_MAX_ELEMENTS];
    barcode_t        capture;
    barcode_cmd_t    cmd    = BARCODE_CMD_NONE;
    barcode_result_t result = BARCODE_NONE;

    /* Feed 'B' one sample at a time, as the task does. */
    s3_test_build(g_test_patterns[1u], widths);
    barcode_init(&capture);

    for (uint32_t element = 0u; (element < BARCODE_MAX_ELEMENTS)
         && (BARCODE_NONE == result); element++)
    {
        bool const b_black = (0u == (element % 2u));

        for (uint32_t sample = 0u; (sample < widths[element])
             && (BARCODE_NONE == result); sample++)
        {
            result = barcode_feed(&capture, b_black, 1u, &cmd);
        }
    }

    for (uint32_t sample = 0u; (sample < S3_TEST_QUIET_SAMPLES)
         && (BARCODE_NONE == result); sample++)
    {
        result = barcode_feed(&capture, false, 1u, &cmd);
    }

    bool const b_decoded = (BARCODE_DECODED == result)
                           && (BARCODE_CMD_RIGHT == cmd);

    /* One bar and then a long white run is a crossing or junction. */
    barcode_init(&capture);
    (void) barcode_feed(&capture, true, S3_TEST_BAR_WIDTH, &cmd);

    bool const b_single = (BARCODE_SINGLE_BAR
                           == barcode_feed(&capture, false,
                                           S3_TEST_LONG_WHITE, &cmd));

    /* More elements than any barcode give one error at the end. */
    barcode_init(&capture);
    result = BARCODE_NONE;

    for (uint32_t run = 0u; (run < S3_TEST_OVERFLOW_RUNS)
         && (BARCODE_NONE == result); run++)
    {
        result = barcode_feed(&capture, (0u == (run % 2u)),
                              S3_TEST_RUN_WIDTH, &cmd);
    }

    if (BARCODE_NONE == result)
    {
        result = barcode_feed(&capture, false, S3_TEST_LONG_WHITE, &cmd);
    }

    return (b_decoded && b_single && (BARCODE_ERROR == result));
}

static bool
s3_test_sampling (void)
{
    uint32_t const first = line_sensor_samples_get();

    tk_dly_tsk(S3_TEST_SAMPLE_WAIT_MS);

    /* Unsigned subtraction stays correct across counter wraparound. */
    return ((line_sensor_samples_get() - first) >= S3_TEST_MIN_SAMPLES);
}

#endif /* APP_TEST */

/* End of file. */

