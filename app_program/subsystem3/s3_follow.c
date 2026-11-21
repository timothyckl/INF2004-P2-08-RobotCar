/**
 * @file s3_follow.c
 * @brief Subsystem 3 mission loop: front line reports and barcode commands.
 *
 * Moved unchanged in behaviour from the S3_LINE_TEST trial in
 * subsystem3_test.c, so the normal firmware runs the mission (S2
 * decision 28). Runs in the initial task, as the trial did.
 */
#include "s3_follow.h"

#include "subsystem3.h"
#include "s3_barcode.h"
#include "s3_ir.h"
#include "s3_ir_gpio.h"
#include "s3_nav.h"
#include "../subsystem2/subsystem2.h"

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
#define S3_LINE_FILTER_SAMPLES        3u
#define S3_LINE_READY_LIMIT_MS     5000u
#define S3_LINE_SAMPLE_MS            10u
#define S3_LINE_REPORT_MS            30u
#define S3_LINE_LOG_MS              500u
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

static UB const g_s3_white[] = "WHITE";
static UB const g_s3_black[] = "BLACK";

static void s3_follow_barcode_sample(s3_barcode_t * p_barcode,
                                     bool * p_read_failed);
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
            (void)tk_dly_tsk(S3_LINE_SAMPLE_MS);
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

        if (!s3_ir_filter(&filter, black_mask, S3_LINE_FILTER_SAMPLES))
        {
            (void)tk_dly_tsk(S3_LINE_SAMPLE_MS);
            continue;
        }

        if (!s3_follow_line_position(s3_ir_follow_decision(filter.stable),
                                   &position))
        {
            tm_printf((UB *)"s3 line: invalid line decision; STOP\n");
            (void)s2_line_stop();
            (void)tk_dly_tsk(S3_LINE_SAMPLE_MS);
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

        (void)tk_dly_tsk(S3_LINE_SAMPLE_MS);
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
        tm_printf((UB *)"s3 bar: DECODED %c (decoded=%u errors=%u)\n",
                  (INT)symbol, (UINT)decoded_count, (UINT)error_count);

        if (S3_NAV_NONE == command)
        {
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
                  p_capture->black ? "BLACK" : "WHITE",
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
