/**
 * @file s3_follow.c
 * @brief Subsystem 3 mission loop: front line reports and barcode commands.
 *
 * Moved unchanged in behaviour from the S3_LINE_TEST trial in
 * subsystem3_test.c, so the normal firmware runs the mission (S2
 * decision 28). Runs in the initial task, as the trial did.
 *
 * Period: S3_IR_SAMPLE_MS (10 ms) from a cyclic handler that wakes the
 * loop, as Motion Control and the Obstacle Scan task do (S3 decision 10).
 * The loop used to sleep with tk_dly_tsk(10); the kernel adds one tick to
 * every delay, so a pass really came every 20 ms (F-01). Wake-ups that
 * arrive while a pass is still running are counted as overruns and
 * dropped, so a slow pass never causes a burst of catch-up passes.
 */
#include "s3_follow.h"

#include "subsystem3.h"
#include "s3_barcode.h"
#include "s3_ir.h"
#include "s3_ir_gpio.h"
#include "s3_nav.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem2/task_health.h"

/* s3_command_t values are sent as S2_NAV_* (subsystem2.h). */
typedef char s3_nav_codes_match_t[
    (((uint8_t)S3_NAV_LEFT == S2_NAV_LEFT)
     && ((uint8_t)S3_NAV_RIGHT == S2_NAV_RIGHT)
     && ((uint8_t)S3_NAV_STRAIGHT == S2_NAV_STRAIGHT)
     && ((uint8_t)S3_NAV_UTURN == S2_NAV_UTURN)) ? 1 : -1];

#ifndef S4_RAW_LOG
#define S4_RAW_LOG 0
#endif

#if S4_RAW_LOG
#include "../subsystem4/subsystem4.h"   /* s4_watch_add (diagnostic) */

/* Line loop count, read by subsystem 4's stall watcher. */
static volatile uint32_t g_s3_line_loops = 0u;
#define S3_LINE_LOOP_DONE()     (g_s3_line_loops++)
#else
#define S3_LINE_LOOP_DONE()     ((void) 0)
#endif

#define S3_LINE_READY_POLL_MS        20u
#define S3_LINE_READY_LIMIT_MS     5000u
/* A front-sensor pattern must hold this long before it is reported: the
 * 60 ms that line following ran with on the car (3 samples at the real
 * 20 ms period, F-01), kept when sampling moved to 10 ms. */
#define S3_LINE_FILTER_MS            60u
#define S3_LINE_FILTER_SAMPLES     (S3_LINE_FILTER_MS / S3_IR_SAMPLE_MS)
#define S3_LINE_REPORT_MS            30u
#define S3_LINE_LOG_MS              500u
/* Guard on the period wait: a lost wake-up costs one late pass, never a
 * stopped loop. Five periods. */
#define S3_LINE_WAKE_LIMIT_MS        50
#define S3_LINE_TIMING_LOG_MS     10000u
#define S3_LEFT_INITIAL_WHITE       136u
#define S3_LEFT_MINIMUM_WHITE        96u
#define S3_LEFT_MAXIMUM_WHITE       168u
/* Left margins clear the tilted white on the hump (S3 decision 8). */
#define S3_LEFT_BLACK_MARGIN        100u
#define S3_LEFT_WHITE_MARGIN         80u
#define S3_LEFT_BLACK_REFERENCE    1500u
#define S3_RIGHT_INITIAL_WHITE      180u
#define S3_RIGHT_MINIMUM_WHITE      100u
#define S3_RIGHT_MAXIMUM_WHITE      300u
#define S3_RIGHT_BLACK_MARGIN       500u
#define S3_RIGHT_WHITE_MARGIN       350u
#define S3_RIGHT_BLACK_REFERENCE   3300u

typedef char s3_line_filter_fits_t[
    ((0u == (S3_LINE_FILTER_MS % S3_IR_SAMPLE_MS))
     && (S3_LINE_FILTER_SAMPLES >= 1u)
     && (S3_LINE_FILTER_SAMPLES <= UINT8_MAX)) ? 1 : -1];

/* Loop timing, kept by the S3 loop only (no other task reads it). */
typedef struct
{
    uint32_t pass_start_us;     /* TIMERAWL when the current pass woke */
    uint32_t worst_pass_us;     /* longest pass since the loop started */
    uint32_t overruns;          /* periods lost to a pass still running */
    uint32_t late_wakes;        /* waits that ended on the guard time-out */
    uint32_t last_log_ms;
    bool     b_started;         /* pass_start_us is valid */
} s3_follow_timing_t;

static UB const g_s3_white[] = "WHITE";
static UB const g_s3_black[] = "BLACK";

/* The task the period handler wakes; written once before it starts. */
static ID g_s3_follow_tid = 0;

static void s3_follow_barcode_sample(s3_barcode_t * p_barcode,
                                     bool * p_read_failed);
static bool s3_follow_start_period(void);
static void s3_follow_cyclic(void * p_exinf);
static void s3_follow_wait_period(s3_follow_timing_t * p_timing);
static bool s3_follow_wait_ready(void);
static uint32_t s3_follow_now_ms(void);
static bool s3_follow_line_position(s3_line_t line, uint8_t * p_position);
static UB const * s3_follow_line_name(uint8_t position);

void
s3_follow_run (void)
{
    static s3_ir_adaptive_config_t const left_config =
    {
        S3_LEFT_INITIAL_WHITE,
        S3_LEFT_MINIMUM_WHITE,
        S3_LEFT_MAXIMUM_WHITE,
        S3_LEFT_BLACK_MARGIN,
        S3_LEFT_WHITE_MARGIN
    };
    static s3_ir_adaptive_config_t const right_config =
    {
        S3_RIGHT_INITIAL_WHITE,
        S3_RIGHT_MINIMUM_WHITE,
        S3_RIGHT_MAXIMUM_WHITE,
        S3_RIGHT_BLACK_MARGIN,
        S3_RIGHT_WHITE_MARGIN
    };
    s3_ir_adaptive_t left_state = {0};
    s3_ir_adaptive_t right_state = {0};
    s3_barcode_t barcode = {0};
    s3_filter_t filter = {0};
    s3_follow_timing_t timing = {0};
    uint32_t last_report_ms = 0u;
    uint32_t last_log_ms = 0u;
    uint8_t last_position = UINT8_MAX;
    uint8_t last_flags = 0u;
    bool b_delivery_failed = false;
    bool b_read_failed = false;
    bool b_barcode_read_failed = false;

    if ((!s3_ir_adaptive_init(&left_state, &left_config))
        || (!s3_ir_adaptive_init(&right_state, &right_config))
        || (!s3_barcode_init(&barcode)))
    {
        tm_printf((UB *)"s3 line: invalid adaptive configuration; STOP\n");
        (void)s2_line_stop();
        return;
    }

    if (!s3_follow_wait_ready())
    {
        tm_printf((UB *)"s3 line: S2 did not reach READY; not started\n");
        (void)s2_line_stop();
        return;
    }

    if (!s3_follow_start_period())
    {
        tm_printf((UB *)"s3 line: period handler not created; STOP\n");
        (void)s2_line_stop();
        return;
    }

    tm_printf((UB *)"s3 line: place the car on the line, then press GP20 "
                    "to START or STOP\n");
    tm_printf((UB *)"s3 line: adaptive AO left=GP26 right=GP27\n");
    tm_printf((UB *)"s3 bar: side AO=GP28, Code 39 A-Z; a mapped letter "
                    "commands the next junction\n");

#if S4_RAW_LOG
    s4_watch_add(tk_get_tid(), &g_s3_line_loops, "s3 line");
#endif

    for (;;)
    {
        s3_follow_wait_period(&timing);
        S3_LINE_LOOP_DONE();    /* each pass, for the stall watcher */

        s3_ir_adc_sample_t sample = {0u, 0u};
        uint8_t black_mask = 0u;
        uint8_t position = S2_LINE_STOP;
        uint8_t strength = 0u;
        uint32_t const now_ms = s3_follow_now_ms();
        ER const read_error = s3_ir_gpio_read_analog(&sample);

        if (E_OK != read_error)
        {
            if (!b_read_failed)
            {
                tm_printf((UB *)"s3 line: ADC read failed, error=%d; STOP\n",
                          read_error);
                (void)s2_line_stop();
                b_read_failed = true;
            }
            continue;
        }
        b_read_failed = false;

        /* Every period, before the steering filter can skip the rest. */
        s3_follow_barcode_sample(&barcode, &b_barcode_read_failed);

        if (s3_ir_adaptive_update(&left_state, sample.left))
        {
            black_mask |= S3_IR_LEFT_MASK;
        }
        if (s3_ir_adaptive_update(&right_state, sample.right))
        {
            black_mask |= S3_IR_RIGHT_MASK;
        }

        /* S3_LINE_FILTER_SAMPLES is 6; s3_line_filter_fits_t keeps it
           within uint8_t. */
        if (!s3_ir_filter(&filter, black_mask,
                          (uint8_t)S3_LINE_FILTER_SAMPLES))
        {
            continue;
        }

        if (!s3_follow_line_position(s3_ir_follow_decision(filter.stable),
                                   &position))
        {
            tm_printf((UB *)"s3 line: invalid line decision; STOP\n");
            (void)s2_line_stop();
            continue;
        }

        if (S2_LINE_LEFT == position)
        {
            strength = s3_ir_adaptive_strength(
                &left_state, S3_LEFT_BLACK_REFERENCE);
        }
        else if (S2_LINE_RIGHT == position)
        {
            strength = s3_ir_adaptive_strength(
                &right_state, S3_RIGHT_BLACK_REFERENCE);
        }
        else
        {
            /* Straight and crossing reports do not carry steering strength. */
        }

        /* S2 decision 25: tell the controller while a barcode is read. */
        uint8_t const flags = s3_barcode_active(&barcode)
                              ? S2_LINE_FLAG_BARCODE : 0u;

        if ((position != last_position) || (flags != last_flags)
            || ((now_ms - last_report_ms) >= S3_LINE_REPORT_MS))
        {
            if (!s2_line_submit_scaled(position, strength, flags, now_ms))
            {
                if (!b_delivery_failed)
                {
                    tm_printf((UB *)"s3 line: report delivery failed; STOP\n");
                }
                (void)s2_line_stop();
                b_delivery_failed = true;
            }
            else
            {
                b_delivery_failed = false;

                if ((position != last_position)
                    || ((now_ms - last_log_ms) >= S3_LINE_LOG_MS))
                {
                    /* Console arguments: ADC values, filters and thresholds
                       are uint16_t (0..4095), and the mask and strength are
                       small unsigned values; UINT (32-bit) holds them all. */
                    tm_printf((UB *)"s3 line: adc=%u/%u filtered=%u/%u "
                                      "threshold=%u/%u left=%s right=%s "
                                      "mask=%u strength=%u command=%s\n",
                              (UINT)sample.left,
                              (UINT)sample.right,
                              (UINT)left_state.filtered,
                              (UINT)right_state.filtered,
                              (UINT)left_state.threshold,
                              (UINT)right_state.threshold,
                              (0u != (filter.stable & S3_IR_LEFT_MASK))
                                  ? g_s3_black : g_s3_white,
                              (0u != (filter.stable & S3_IR_RIGHT_MASK))
                                  ? g_s3_black : g_s3_white,
                              (UINT)filter.stable,
                              (UINT)strength,
                              s3_follow_line_name(position));
                    last_log_ms = now_ms;
                }
                last_position = position;
                last_flags = flags;
                last_report_ms = now_ms;
            }
        }
    }
}

/*
 * Read the side barcode sensor once and report any finished capture. A
 * decoded letter that maps to a command goes to the Vehicle Controller,
 * which acts at the next junction (S2 decision 26); the barcode never
 * steers directly. A failed read abandons the capture but does not stop
 * line following, which uses the other two channels.
 */
static void
s3_follow_barcode_sample (s3_barcode_t * p_barcode, bool * p_read_failed)
{
    static uint16_t decoded_count = 0u;
    static uint16_t error_count = 0u;
    uint16_t adc = 0u;
    char symbol = '\0';
    s3_scan_result_t result;
    ER const read_error = s3_ir_gpio_read_barcode(&adc);
    uint32_t const now_us = (uint32_t)in_w(TIMER_TIMERAWL);

    if (E_OK != read_error)
    {
        if (!*p_read_failed)
        {
            tm_printf((UB *)"s3 bar: ADC2 read failed, error=%d; capture "
                            "abandoned\n", read_error);
        }
        s3_barcode_abort(p_barcode);
        *p_read_failed = true;
        return;
    }
    *p_read_failed = false;

    result = s3_barcode_update(p_barcode, adc, now_us, &symbol);

    if (S3_SCAN_DECODED == result)
    {
        s3_command_t const command = s3_nav_command(symbol);

        decoded_count++;
        /* Console arguments: symbol is a char A..Z, which INT holds for %c;
           the counts are uint16_t, which UINT holds. */
        tm_printf((UB *)"s3 bar: DECODED %c (decoded=%u errors=%u)\n",
                  (INT)symbol, (UINT)decoded_count, (UINT)error_count);

        if (S3_NAV_NONE == command)
        {
            /* symbol is a char A..Z, which INT holds for %c. */
            tm_printf((UB *)"s3 bar: no command mapped to %c; reported "
                            "only\n", (INT)symbol);
        }

        /* Every letter goes to the controller for telemetry; it acts
           only on a mapped command (S2 decision 27). */
        if (!s2_nav_submit((uint8_t)command, symbol, decoded_count,
                           s3_follow_now_ms()))
        {
            tm_printf((UB *)"s3 bar: %c not delivered\n", (INT)symbol);
        }
    }
    else if (S3_SCAN_ERROR == result)
    {
        s3_capture_t const * const p_capture = &p_barcode->scan.capture;

        error_count++;
        /* Console arguments: the run count is at most 31, the threshold a
           12-bit ADC value and the counts uint16_t, so UINT holds them all. */
        tm_printf((UB *)"s3 bar: capture rejected, runs=%u threshold=%u "
                        "(decoded=%u errors=%u)\n",
                  (UINT)p_capture->count,
                  (UINT)p_barcode->level.threshold,
                  (UINT)decoded_count, (UINT)error_count);

        /* Diagnostic: stored runs in ms, first run black, then the level
         * and age of the run that was still open when the capture ended. */
        for (uint8_t idx = 0u; idx < p_capture->count; idx += 8u)
        {
            tm_printf((UB *)"s3 bar:  runs %u..:", (UINT)idx);
            for (uint8_t run = idx;
                 (run < p_capture->count) && (run < (idx + 8u)); run++)
            {
                tm_printf((UB *)" %u",
                          (UINT)(p_capture->runs[run] / 1000u));
            }
            tm_printf((UB *)"\n");
        }
        tm_printf((UB *)"s3 bar:  open run %s for %u ms\n",
                  p_capture->b_black ? "BLACK" : "WHITE",
                  (UINT)((now_us - p_capture->last_us) / 1000u));
    }
    else if (S3_SCAN_SINGLE_BAR == result)
    {
        tm_printf((UB *)"s3 bar: single bar (crossing line), ignored\n");
    }
    else
    {
        /* No capture finished in this period. */
    }
}

/* Create and start the period handler that wakes this loop. */
static bool
s3_follow_start_period (void)
{
    T_CCYC ccyc = {0};

    g_s3_follow_tid = tk_get_tid();
    ccyc.cycatr = TA_HLNG | TA_STA;
    /* FP is the kernel's generic handler type; the cast changes only the
       pointer type. */
    ccyc.cychdr = (FP)s3_follow_cyclic;
    ccyc.cyctim = S3_IR_SAMPLE_MS;
    ccyc.cycphs = S3_IR_SAMPLE_MS;

    return ((g_s3_follow_tid > 0) && (tk_cre_cyc(&ccyc) >= E_OK));
}

/* Cyclic handler, every S3_IR_SAMPLE_MS: wake the loop. */
static void
s3_follow_cyclic (void * p_exinf)
{
    (void)p_exinf;

    if (g_s3_follow_tid > 0)
    {
        (void)tk_wup_tsk(g_s3_follow_tid);
    }
}

/*
 * End one pass and wait for the next period. Wake-ups queued while the
 * pass ran are periods it overran: they are counted and dropped, so the
 * next pass starts on the next period boundary. The worst pass time and
 * both counters are printed every S3_LINE_TIMING_LOG_MS.
 */
static void
s3_follow_wait_period (s3_follow_timing_t * p_timing)
{
    /* TIMERAWL is the 32-bit low word of the 1 MHz timer (UW): same
       width, no value changes. */
    uint32_t const end_us = (uint32_t)in_w(TIMER_TIMERAWL);
    /* Before the first pass, wake-ups queued during start-up are not
       overruns; they are dropped all the same. */
    INT const queued = tk_can_wup(TSK_SELF);
    uint32_t now_ms;

    if (p_timing->b_started)
    {
        /* Modulo 2^32 difference: correct across the timer wrap. */
        uint32_t const pass_us = end_us - p_timing->pass_start_us;

        p_timing->worst_pass_us = (pass_us > p_timing->worst_pass_us)
                                  ? pass_us : p_timing->worst_pass_us;

        if (queued > 0)
        {
            /* queued is a positive wake-up count, which uint32_t holds;
               the counter would need over a year of overruns to wrap. */
            p_timing->overruns += (uint32_t)queued;
        }
    }

    if (E_TMOUT == tk_slp_tsk(S3_LINE_WAKE_LIMIT_MS))
    {
        p_timing->late_wakes++;
    }

    /* As above: the 32-bit timer word, unchanged. */
    p_timing->pass_start_us = (uint32_t)in_w(TIMER_TIMERAWL);
    p_timing->b_started = true;
    s2_health_check_in(S2_HEALTH_LINE);     /* S2 decision 37 */

    now_ms = s3_follow_now_ms();
    if ((now_ms - p_timing->last_log_ms) >= S3_LINE_TIMING_LOG_MS)
    {
        /* Counters and the pass time (us) are uint32_t; UINT is 32-bit. */
        tm_printf((UB *)"s3 loop: period %u ms, worst pass %u us, "
                        "overruns %u, late wakes %u\n",
                  (UINT)S3_IR_SAMPLE_MS, (UINT)p_timing->worst_pass_us,
                  (UINT)p_timing->overruns, (UINT)p_timing->late_wakes);
        p_timing->last_log_ms = now_ms;
    }
}

static bool
s3_follow_wait_ready (void)
{
    uint32_t const start_ms = s3_follow_now_ms();
    s2_robot_state_t state;

    while ((s3_follow_now_ms() - start_ms) < S3_LINE_READY_LIMIT_MS)
    {
        if ((E_OK == s2_get_robot_state(&state))
            && (S2_SYS_READY == state.state))
        {
            return (true);
        }
        (void)tk_dly_tsk(S3_LINE_READY_POLL_MS);
    }

    return (false);
}

static uint32_t
s3_follow_now_ms (void)
{
    SYSTIM time;

    /* time.lo is the 32-bit low word of the ms clock (UW): same width, no
       value changes; it wraps every 49.7 days. */
    return ((tk_get_tim(&time) >= E_OK) ? (uint32_t)time.lo : 0u);
}

static bool
s3_follow_line_position (s3_line_t line, uint8_t * p_position)
{
    if (NULL == p_position)
    {
        return (false);
    }

    if (S3_LINE_STRAIGHT == line)
    {
        *p_position = S2_LINE_CENTER;
    }
    else if (S3_LINE_LEFT == line)
    {
        *p_position = S2_LINE_LEFT;
    }
    else if (S3_LINE_RIGHT == line)
    {
        *p_position = S2_LINE_RIGHT;
    }
    else if (S3_LINE_JUNCTION == line)
    {
        *p_position = S2_LINE_BOTH_BLACK;
    }
    else
    {
        return (false);
    }

    return (true);
}

static UB const *
s3_follow_line_name (uint8_t position)
{
    static UB const straight[] = "STRAIGHT";
    static UB const left[] = "LEFT";
    static UB const right[] = "RIGHT";
    static UB const invalid[] = "INVALID";
    UB const * p_name = invalid;

    if (S2_LINE_CENTER == position)
    {
        p_name = straight;
    }
    else if (S2_LINE_LEFT == position)
    {
        p_name = left;
    }
    else if (S2_LINE_RIGHT == position)
    {
        p_name = right;
    }
    else if (S2_LINE_STOP == position)
    {
        static UB const stop[] = "STOP";

        p_name = stop;
    }
    else if (S2_LINE_BOTH_BLACK == position)
    {
        static UB const crossing[] = "CROSSING";

        p_name = crossing;
    }
    else
    {
        /* Retain INVALID for values that cannot command motion. */
    }

    return (p_name);
}

/* End of file. */
