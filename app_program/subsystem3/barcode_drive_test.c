/**
 * @file barcode_drive_test.c
 * @brief Controlled line-follow drive while observing barcode decoding.
 */

#include "barcode_drive_test.h"

#include "s3_bar.h"
#include "s3_config.h"
#include "subsystem3.h"
#include "../subsystem2/motion_api.h"
#include "../subsystem2/s2_s1_adapter.h"
#include "../subsystem2/subsystem2.h"

#include <stdbool.h>
#include <stdint.h>
#include <sys/sysdef.h>
#include <tk/syslib.h>

#ifndef S3_BARCODE_DRIVE_DIAGNOSTIC
#define S3_BARCODE_DRIVE_DIAGNOSTIC 0
#endif

#if S3_BARCODE_DRIVE_DIAGNOSTIC

#define S3_BAR_DRIVE_TASK_PRI          17
#define S3_BAR_DRIVE_PERIOD_MS         UINT32_C(10)
#define S3_BAR_DRIVE_LOG_MS            UINT32_C(100)
#define S3_BAR_DRIVE_DEBOUNCE_SAMPLES  UINT8_C(3)
#define S3_BAR_DRIVE_START_RETRY_MS     UINT32_C(100)
#define S3_BAR_DRIVE_START_TIMEOUT_MS   UINT32_C(2000)
#define S3_BAR_DRIVE_US_PER_MS           UINT32_C(1000)
#define S3_BAR_DRIVE_RUNS_PER_ROW        UINT8_C(8)

typedef struct
{
    uint32_t runs_ms[S3_BAR_CAPACITY];
    uint8_t count;
    bool b_active;
    bool b_overflow;
} s3_barcode_drive_frame_t;

static bool     gb_start_pending = false;
static uint8_t  g_start_attempts = 0U;
static uint32_t g_start_begin_ms = 0U;
static uint32_t g_start_request_ms = 0U;

static void s3_barcode_drive_task(INT start_code, void * p_extension);
static void s3_barcode_drive_button_init(void);
static bool s3_barcode_drive_button_event(void);
static void s3_barcode_drive_button_apply(uint32_t now_ms);
static void s3_barcode_drive_start_poll(
    uint32_t now_ms, s3_line_status_t const * p_line);
static void s3_barcode_drive_log(s3_line_status_t const * p_line);
static void s3_barcode_drive_frame_edge(
    s3_barcode_drive_frame_t * p_frame, bool b_previous_black,
    bool b_black, uint32_t duration_ms);
static void s3_barcode_drive_frame_poll(
    s3_barcode_drive_frame_t * p_frame, bool b_black,
    uint32_t white_ms, uint32_t quiet_ms);
static void s3_barcode_drive_frame_dump(
    s3_barcode_drive_frame_t const * p_frame);
static char const * s3_barcode_drive_colour(bool b_black);

ER
s3_barcode_drive_test_start (void)
{
    s3_barcode_drive_button_init();

    ID const task_id = app_start_task((FP)s3_barcode_drive_task,
                                      S3_BAR_DRIVE_TASK_PRI, 0U,
                                      "barcode_drive");

    return ((task_id < E_OK) ? task_id : E_OK);
}

static void
s3_barcode_drive_task (INT start_code, void * p_extension)
{
    s3_line_status_t line = {0};
    uint32_t last_log_ms = 0U;
    uint8_t last_barcodes = 0U;
    uint8_t last_errors = 0U;
    bool b_last_black = false;
    bool b_have_barcode = false;
    bool b_have_line = false;
    uint32_t last_transition_ms = 0U;
    uint32_t last_line_seq = 0U;
    s3_barcode_drive_frame_t frame = {{0U}, 0U, false, false};

    (void)start_code;
    (void)p_extension;

    tm_printf((UB *)"[S3 BAR DRIVE] GP20 ready: "
              "START, STOP, or RESET before restart\n");
    tm_printf((UB *)"[S3 BAR DRIVE] controller follows the line while "
              "the ADC2/GP28 decoder runs\n");

    for (;;)
    {
        uint32_t const now_ms = s2_clock_ms();

        if (s3_barcode_drive_button_event())
        {
            s3_barcode_drive_button_apply(now_ms);
        }

        if ((E_OK == s3_line_get(&line))
            && ((!b_have_line) || (line.seq != last_line_seq)))
        {
            bool const b_black =
                (0U != (line.flags & S3_ST_BAR_BLACK));

            if (!b_have_line)
            {
                last_barcodes = line.barcodes;
                last_errors = line.barcode_errors;
                b_have_line = true;
                last_line_seq = line.seq;
            }

            if (!b_have_barcode)
            {
                b_last_black = b_black;
                last_transition_ms = line.captured_ms;
                b_have_barcode = true;
            }
            else if (b_black != b_last_black)
            {
                uint32_t const duration_ms =
                    line.captured_ms - last_transition_ms;

                tm_printf((UB *)"[S3 BAR RAW] run=%s duration=%u ms "
                          "-> %s adc=%u threshold=%u\n",
                          (UB *)s3_barcode_drive_colour(b_last_black),
                          (UINT)duration_ms,
                          (UB *)s3_barcode_drive_colour(b_black),
                          (UINT)line.barcode_adc,
                          (UINT)line.barcode_threshold);
                s3_barcode_drive_frame_edge(&frame, b_last_black, b_black,
                                            duration_ms);
                b_last_black = b_black;
                last_transition_ms = line.captured_ms;
            }

            s3_barcode_drive_frame_poll(
                &frame, b_black, line.captured_ms - last_transition_ms,
                S3_BAR_QUIET_US / S3_BAR_DRIVE_US_PER_MS);

            if (line.barcodes != last_barcodes)
            {
                tm_printf((UB *)"[S3 BAR RESULT] DECIPHERED=%c "
                          "successful_scans=%u\n",
                          (INT)line.last_symbol, (UINT)line.barcodes);
                last_barcodes = line.barcodes;
            }

            if (line.barcode_errors != last_errors)
            {
                tm_printf((UB *)"[S3 BAR DRIVE] DECODE ERROR total=%u\n",
                          (UINT)line.barcode_errors);
                last_errors = line.barcode_errors;
            }

            if ((now_ms - last_log_ms) >= S3_BAR_DRIVE_LOG_MS)
            {
                s3_barcode_drive_log(&line);
                last_log_ms = now_ms;
            }

            last_line_seq = line.seq;
        }

        s3_barcode_drive_start_poll(now_ms,
                                    b_have_line ? &line : NULL);

        (void)tk_dly_tsk((RELTIM)S3_BAR_DRIVE_PERIOD_MS);
    }
}

static void
s3_barcode_drive_button_init (void)
{
    uint32_t const mask = UINT32_C(1) << S3_PIN_BARCODE_DRIVE_BUTTON;

    out_w(GPIO_OE_CLR, mask);
    out_w(GPIO_CTRL(S3_PIN_BARCODE_DRIVE_BUTTON), GPIO_CTRL_FUNCSEL_SIO);
    out_w(GPIO(S3_PIN_BARCODE_DRIVE_BUTTON),
          GPIO_IE | GPIO_SHEMITT | GPIO_PUE);
}

static bool
s3_barcode_drive_button_event (void)
{
    static uint8_t samples = 0U;
    static bool b_latched = false;
    uint32_t const mask = UINT32_C(1) << S3_PIN_BARCODE_DRIVE_BUTTON;
    bool const b_pressed = (0U == (in_w(GPIO_IN) & mask));
    bool b_event = false;

    if (b_pressed)
    {
        if (samples < S3_BAR_DRIVE_DEBOUNCE_SAMPLES)
        {
            samples++;
        }

        if ((samples >= S3_BAR_DRIVE_DEBOUNCE_SAMPLES) && (!b_latched))
        {
            b_latched = true;
            b_event = true;
        }
    }
    else
    {
        samples = 0U;
        b_latched = false;
    }

    return (b_event);
}

static void
s3_barcode_drive_button_apply (uint32_t now_ms)
{
    s2_robot_state_t state;

    if (E_OK != s2_get_robot_state(&state))
    {
        tm_printf((UB *)"[S3 BAR DRIVE] GP20 ignored: state unavailable\n");
    }
    else if (S2_SYS_READY == state.state)
    {
        gb_start_pending = true;
        g_start_attempts = 0U;
        g_start_begin_ms = now_ms;
        g_start_request_ms = now_ms - S3_BAR_DRIVE_START_RETRY_MS;
        tm_printf((UB *)"[S3 BAR DRIVE] GP20: START armed; "
                  "waiting for valid line\n");
    }
    else if (S2_SYS_RUNNING == state.state)
    {
        gb_start_pending = false;
        bool const b_sent = controller_raise_remote_stop(NULL, now_ms);

        tm_printf((UB *)(b_sent
                  ? "[S3 BAR DRIVE] GP20: STOP requested\n"
                  : "[S3 BAR DRIVE] GP20: STOP request failed\n"));
    }
    else if ((S2_SYS_STOPPED == state.state)
             || (S2_SYS_FAULT == state.state))
    {
        gb_start_pending = false;
        bool const b_sent = controller_send_remote_command(
            NULL, S2_REMOTE_RESET, now_ms);

        tm_printf((UB *)(b_sent
                  ? "[S3 BAR DRIVE] GP20: RESET requested; "
                    "press again when READY\n"
                  : "[S3 BAR DRIVE] GP20: RESET request failed\n"));
    }
    else
    {
        gb_start_pending = false;
        tm_printf((UB *)"[S3 BAR DRIVE] GP20 ignored: "
                  "controller state=%u\n", (UINT)state.state);
    }
}

static void
s3_barcode_drive_start_poll (
    uint32_t now_ms, s3_line_status_t const * p_line)
{
    s2_robot_state_t state;

    if ((!gb_start_pending) || (E_OK != s2_get_robot_state(&state)))
    {
        return;
    }

    if (S2_SYS_RUNNING == state.state)
    {
        gb_start_pending = false;
        tm_printf((UB *)"[S3 BAR DRIVE] START accepted after %u "
                  "request(s)\n", (UINT)g_start_attempts);
    }
    else if (S2_SYS_READY != state.state)
    {
        gb_start_pending = false;
        tm_printf((UB *)"[S3 BAR DRIVE] START cancelled: state=%u\n",
                  (UINT)state.state);
    }
    else if ((now_ms - g_start_begin_ms)
             >= S3_BAR_DRIVE_START_TIMEOUT_MS)
    {
        uint8_t const flags = (NULL != p_line) ? p_line->flags : 0U;
        uint8_t const line = (NULL != p_line) ? p_line->line : 0U;

        gb_start_pending = false;
        tm_printf((UB *)"[S3 BAR DRIVE] START refused: line=%u "
                  "flags=0x%x valid=%s; check front AO GP26/GP27\n",
                  (UINT)line, (UINT)flags,
                  (UB *)((0U != (flags & S3_ST_VALID)) ? "YES" : "NO"));
    }
    else if (((now_ms - g_start_request_ms)
              >= S3_BAR_DRIVE_START_RETRY_MS)
             && (NULL != p_line)
             && (0U != (p_line->flags & S3_ST_VALID)))
    {
        bool const b_sent = controller_send_remote_command(
            NULL, S2_REMOTE_START, now_ms);

        g_start_request_ms = now_ms;
        if (b_sent)
        {
            g_start_attempts++;
        }
    }
    else
    {
        /* Wait for a valid line snapshot or the next retry period. */
    }
}

static void
s3_barcode_drive_log (s3_line_status_t const * p_line)
{
    motion_status_t motion;
    s2_robot_state_t state;
    ER const motion_error = motion_get_status(&motion);
    ER const state_error = s2_get_robot_state(&state);
    char const * const p_colour =
        (0U != (p_line->flags & S3_ST_BAR_BLACK)) ? "BLACK" : "WHITE";
    INT const last_symbol = (0U != p_line->last_symbol)
                            ? (INT)p_line->last_symbol : (INT)'-';

    if ((E_OK == motion_error) && (E_OK == state_error))
    {
        tm_printf((UB *)"[S3 BAR DRIVE] state=%u line=%u flags=0x%x "
                  "barcode=%s adc=%u threshold=%u speed=%d/%d mm/s "
                  "letter=%c decoded=%u errors=%u\n",
                  (UINT)state.state, (UINT)p_line->line,
                  (UINT)p_line->flags, (UB *)p_colour,
                  (UINT)p_line->barcode_adc,
                  (UINT)p_line->barcode_threshold,
                  (INT)motion.left_mm_s, (INT)motion.right_mm_s,
                  last_symbol,
                  (UINT)p_line->barcodes,
                  (UINT)p_line->barcode_errors);
    }
}

static void
s3_barcode_drive_frame_edge (
    s3_barcode_drive_frame_t * p_frame, bool b_previous_black,
    bool b_black, uint32_t duration_ms)
{
    if (!p_frame->b_active)
    {
        if ((!b_previous_black) && b_black)
        {
            p_frame->count = 0U;
            p_frame->b_overflow = false;
            p_frame->b_active = true;
        }
    }
    else if (p_frame->count < S3_BAR_CAPACITY)
    {
        p_frame->runs_ms[p_frame->count] = duration_ms;
        p_frame->count++;
    }
    else
    {
        p_frame->b_overflow = true;
    }
}

static void
s3_barcode_drive_frame_poll (
    s3_barcode_drive_frame_t * p_frame, bool b_black,
    uint32_t white_ms, uint32_t quiet_ms)
{
    if (p_frame->b_active && (!b_black) && (white_ms >= quiet_ms))
    {
        s3_barcode_drive_frame_dump(p_frame);
        p_frame->b_active = false;
    }
}

static void
s3_barcode_drive_frame_dump (
    s3_barcode_drive_frame_t const * p_frame)
{
    char pattern[S3_BAR_CAPACITY + 1U];
    uint32_t narrowest = UINT32_MAX;
    uint32_t widest = 0U;

    for (uint8_t idx = 0U; idx < p_frame->count; idx++)
    {
        uint32_t const run = p_frame->runs_ms[idx];

        narrowest = (run < narrowest) ? run : narrowest;
        widest = (run > widest) ? run : widest;
    }

    uint32_t const threshold = (narrowest / 2U) + (widest / 2U)
                               + ((narrowest & widest) & 1U);

    for (uint8_t idx = 0U; idx < p_frame->count; idx++)
    {
        pattern[idx] = (p_frame->runs_ms[idx] > threshold) ? 'W' : 'N';
    }
    pattern[p_frame->count] = '\0';

    tm_printf((UB *)"[S3 BAR FRAME] runs=%u expected=%u overflow=%s "
              "min=%u max=%u split=%u ms pattern=%s\n",
              (UINT)p_frame->count, (UINT)S3_BAR_FRAME_RUNS,
              (UB *)(p_frame->b_overflow ? "YES" : "NO"),
              (UINT)narrowest, (UINT)widest, (UINT)threshold,
              (UB *)pattern);

    for (uint8_t idx = 0U; idx < p_frame->count; idx++)
    {
        if (0U == (idx % S3_BAR_DRIVE_RUNS_PER_ROW))
        {
            tm_printf((UB *)"[S3 BAR FRAME] ms:");
        }

        tm_printf((UB *)" %u", (UINT)p_frame->runs_ms[idx]);

        if (((idx + 1U) == p_frame->count)
            || (0U == ((idx + 1U) % S3_BAR_DRIVE_RUNS_PER_ROW)))
        {
            tm_printf((UB *)"\n");
        }
    }
}

static char const *
s3_barcode_drive_colour (bool b_black)
{
    return b_black ? "BLACK" : "WHITE";
}

#endif /* S3_BARCODE_DRIVE_DIAGNOSTIC */

