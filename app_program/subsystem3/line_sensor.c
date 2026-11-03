/**
 * @file line_sensor.c
 * @brief Line Sensor task: line position, line events and barcode
 *        commands for the Vehicle Controller (IF-06).
 *
 * Every LINE_SENSOR_PERIOD_MS the sampling clock wakes the task. The task
 * samples the IR sensors, updates line tracking and barcode capture,
 * publishes the status word and raises events. It only reports: the
 * Vehicle Controller decides what to do, and only Motion Control drives
 * the motors (Design Review sections 3 and 6.3).
 */
#include "line_sensor.h"

#include "../app.h"
#include "barcode.h"
#include "drv_ir.h"
#include "line_track.h"

/* 1 prints the raw sensor bits and every event, for calibration (S3-U10).
   Keep it 0 for normal and integration runs.
   NOTE: This is a calibration mode, not NDEBUG debug output, because the
   build always defines NDEBUG (build_make/pico_rp2040.mk). */
#ifndef LINE_SENSOR_CALIBRATION
# define LINE_SENSOR_CALIBRATION    (1)
#endif

#define LINE_SENSOR_PRINT_SAMPLES   (20u)       /* Every 200 ms.          */
#define LINE_SENSOR_MISSED_LIMIT    (65534)     /* UINT16_MAX - 1.        */

/* C99 compile-time checks: the array size is negative if a check fails.
   The casts are safe: the enumerators are small non-negative constants. */
typedef uint8_t line_sensor_cmd_check_t
    [((uint32_t) BARCODE_CMD_UTURN <= LINE_SENSOR_STATUS_CMD_MASK) ? 1 : -1];
typedef uint8_t line_sensor_mode_check_t
    [((uint32_t) LINE_TRACK_FAULT <= LINE_SENSOR_STATUS_MODE_MASK) ? 1 : -1];

static ID volatile       gh_line_task   = 0;    /* Read by the clock ISR. */
static ID volatile       gh_line_events = 0;    /* Read by two tasks.     */
static ID                gh_line_clock  = 0;
static uint16_t volatile g_status       = 0u;
static uint32_t volatile g_samples      = 0u;

/** The line sensor task body. */
static void line_sensor_task (INT stacd, void * p_exinf);

/** Sampling clock handler. It runs in interrupt context. */
static void line_sensor_tick_isr (void * p_exinf);

/** Bar level for barcode capture. */
static bool line_sensor_is_bar (uint8_t bits);

/** Convert missed wake-ups to the sample's width in periods. */
static uint16_t line_sensor_elapsed (INT missed);

/** Pack one sample into the status word. */
static uint16_t line_sensor_pack (uint8_t bits,
                                  line_track_t const * p_track,
                                  barcode_cmd_t cmd);

/** Map line tracking events to LINE_SENSOR_EVENT_* bits. */
static uint32_t line_sensor_track_events (uint8_t track_events);

/** Map a barcode capture result to LINE_SENSOR_EVENT_* bits. */
static uint32_t line_sensor_capture_events (barcode_result_t result);

#if LINE_SENSOR_CALIBRATION
/** Print the calibration output for one sample. */
static void line_sensor_print (uint8_t bits, line_track_t const * p_track,
                               uint32_t events, barcode_cmd_t cmd);
#endif

ER
line_sensor_init (void)
{
    T_CFLG cflg = {0};
    T_CCYC ccyc = {0};
    ER     err  = E_OK;

    /* TA_WSGL allows a single waiter, which enforces one consumer. */
    cflg.flgatr    = (TA_TFIFO | TA_WSGL);
    cflg.iflgptn   = 0u;
    gh_line_events = tk_cre_flg(&cflg);

    if (gh_line_events < E_OK)
    {
        err = gh_line_events;
    }
    else
    {
        /* The clock is created stopped. line_sensor_start() starts it
           once the task it wakes exists. */
        ccyc.cycatr   = TA_HLNG;
        ccyc.cychdr   = line_sensor_tick_isr;
        ccyc.cyctim   = LINE_SENSOR_PERIOD_MS;
        ccyc.cycphs   = LINE_SENSOR_PERIOD_MS;
        gh_line_clock = tk_cre_cyc(&ccyc);

        err = (gh_line_clock < E_OK) ? gh_line_clock : E_OK;
    }

    return (err);
}

ER
line_sensor_start (PRI priority)
{
    ER       err    = E_OK;
    ID const h_task = app_start_task(line_sensor_task, priority, 0u,
                                     "line_sensor_task");

    if (h_task < E_OK)
    {
        err = h_task;
    }
    else
    {
        gh_line_task = h_task;
        err          = tk_sta_cyc(gh_line_clock);
    }

    return (err);
}

uint16_t
line_sensor_status_get (void)
{
    return (g_status);
}

uint32_t
line_sensor_samples_get (void)
{
    return (g_samples);
}

ER
line_sensor_events_wait (uint32_t * p_events, TMO timeout)
{
    ER   err     = E_PAR;
    UINT pattern = 0u;

    if (NULL != p_events)
    {
        /* TWF_BITCLR clears only the bits that satisfied this wait. */
        err = tk_wai_flg(gh_line_events, LINE_SENSOR_EVENT_ALL,
                         (TWF_ORW | TWF_BITCLR), &pattern, timeout);

        *p_events = (E_OK == err) ? (pattern & LINE_SENSOR_EVENT_ALL) : 0u;
    }

    return (err);
}

static void
line_sensor_task (INT stacd, void * p_exinf)
{
    line_track_t  track;
    barcode_t     capture;
    barcode_cmd_t last_cmd = BARCODE_CMD_NONE;
    ER const      err      = drv_ir_init();

    /* The start code and extended information are not used. */
    (void) stacd;
    (void) p_exinf;

    if (E_OK != err)
    {
        /* Without its pins the task cannot sense, so report and stop. */
        CHECK(err, "drv_ir_init");
        (void) tk_set_flg(gh_line_events, LINE_SENSOR_EVENT_FAULT);
        tk_ext_tsk();
    }

    line_track_init(&track);
    barcode_init(&capture);

    for (;;)
    {
        barcode_cmd_t cmd = BARCODE_CMD_NONE;

        (void) tk_slp_tsk(TMO_FEVR);

        /* Wake-ups queued while this task was late are counted rather
           than replayed, so bar widths stay in true sample periods. */
        uint16_t const         elapsed =
            line_sensor_elapsed(tk_can_wup(TSK_SELF));
        uint8_t const          bits    = drv_ir_read();
        uint32_t               events  =
            line_sensor_track_events(line_track_update(&track, bits));
        barcode_result_t const result  =
            barcode_feed(&capture, line_sensor_is_bar(bits), elapsed, &cmd);

        if (BARCODE_DECODED == result)
        {
            last_cmd = cmd;
        }

        events |= line_sensor_capture_events(result);

        /* Publish the status before the events that announce it. */
        g_status = line_sensor_pack(bits, &track, last_cmd);
        g_samples++;

        if (0u != events)
        {
            (void) tk_set_flg(gh_line_events, events);
        }

#if LINE_SENSOR_CALIBRATION
        line_sensor_print(bits, &track, events, last_cmd);
#endif
    }
}

static void
line_sensor_tick_isr (void * p_exinf)
{
    /* The extended information is not used. */
    (void) p_exinf;
    (void) tk_wup_tsk(gh_line_task);
}

static bool
line_sensor_is_bar (uint8_t bits)
{
    /* NOTE: Bars are assumed to cover all three sensors, and spaces leave
       at most the line under the centre sensor. Change only this function
       once the barcode placement on the course is confirmed. */
    return (LINE_TRACK_BITS_ALL == (bits & LINE_TRACK_BITS_ALL));
}

static uint16_t
line_sensor_elapsed (INT missed)
{
    uint16_t elapsed = 1u;

    if (missed >= LINE_SENSOR_MISSED_LIMIT)
    {
        elapsed = UINT16_MAX;
    }
    else if (missed > 0)
    {
        /* The cast is safe: missed is 1 to 65533 here, so missed + 1 fits
           in 16 bits. */
        elapsed = (uint16_t) (missed + 1);
    }
    else
    {
        /* With no missed wake-ups, or an error, count this sample only. */
    }

    return (elapsed);
}

static uint16_t
line_sensor_pack (uint8_t bits, line_track_t const * p_track,
                  barcode_cmd_t cmd)
{
    /* The casts are safe: position is -2 to +2, so the offset value is 0
       to 4, and the enumerators are checked against their masks above. */
    uint32_t const ir_bits   = bits;
    uint32_t const position  =
        (uint32_t) (p_track->position + LINE_SENSOR_STATUS_POS_OFFSET);
    uint32_t const valid_bit = p_track->b_valid ? 1u : 0u;
    uint32_t const mode      = (uint32_t) p_track->mode;
    uint32_t const command   = (uint32_t) cmd;
    uint32_t const status    =
        ((ir_bits & LINE_SENSOR_STATUS_IR_MASK) << LINE_SENSOR_STATUS_IR_SHIFT)
        | ((position & LINE_SENSOR_STATUS_POS_MASK)
           << LINE_SENSOR_STATUS_POS_SHIFT)
        | (valid_bit << LINE_SENSOR_STATUS_VALID_SHIFT)
        | ((mode & LINE_SENSOR_STATUS_MODE_MASK)
           << LINE_SENSOR_STATUS_MODE_SHIFT)
        | ((command & LINE_SENSOR_STATUS_CMD_MASK)
           << LINE_SENSOR_STATUS_CMD_SHIFT);

    /* The cast is safe: every field lies in bits 0 to 11. */
    return ((uint16_t) status);
}

static uint32_t
line_sensor_track_events (uint8_t track_events)
{
    uint32_t events = 0u;

    if (0u != (track_events & LINE_TRACK_EVENT_LOST))
    {
        events |= LINE_SENSOR_EVENT_LOST;
    }

    if (0u != (track_events & LINE_TRACK_EVENT_FOUND))
    {
        events |= LINE_SENSOR_EVENT_FOUND;
    }

    if (0u != (track_events & LINE_TRACK_EVENT_FAULT))
    {
        events |= LINE_SENSOR_EVENT_FAULT;
    }

    return (events);
}

static uint32_t
line_sensor_capture_events (barcode_result_t result)
{
    uint32_t events = 0u;

    switch (result)
    {
        case BARCODE_DECODED:
            events = LINE_SENSOR_EVENT_BARCODE;
        break;

        case BARCODE_SINGLE_BAR:
            events = LINE_SENSOR_EVENT_JUNCTION;
        break;

        case BARCODE_ERROR:
            events = LINE_SENSOR_EVENT_BARCODE_ERR;
        break;

        default:
            /* No capture finished on this sample. */
        break;
    }

    return (events);
}

#if LINE_SENSOR_CALIBRATION
static void
line_sensor_print (uint8_t bits, line_track_t const * p_track,
                   uint32_t events, barcode_cmd_t cmd)
{
    /* The UB * casts are safe: tm_printf() reads the ASCII format string
       and does not modify it. */
    if (0u != events)
    {
        tm_printf((UB *) "s3: events 0x%02x cmd=%d\n", events, cmd);
    }

    if (0u == (g_samples % LINE_SENSOR_PRINT_SAMPLES))
    {
        /*
        tm_printf((UB *) "s3: IR L=%d C=%d R=%d pos=%d valid=%d mode=%d\n",
                (0u != (bits & LINE_TRACK_BIT_LEFT)) ? 1 : 0,
                (0u != (bits & LINE_TRACK_BIT_CENTRE)) ? 1 : 0,
                (0u != (bits & LINE_TRACK_BIT_RIGHT)) ? 1 : 0,
                p_track->position, p_track->b_valid ? 1 : 0,
                p_track->mode);
        */
    }
}
#endif

/* End of file. */

