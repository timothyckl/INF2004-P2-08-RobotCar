/** @file task_line.c Subsystem 3 Line Sensor task (IF-06 publisher).
 *
 * Every S3_PERIOD_MS a cyclic handler wakes the task, which:
 *   1. moves barcode edges from the interrupt ring into s3_scan;
 *   2. finishes or expires a barcode capture, staging decoded commands;
 *   3. samples the two front sensors in one GPIO_IN read and runs
 *      s3_track (debounce, decision table, loss, fault, steering);
 *   4. offers a staged command at a junction (s3_nav_take) when robot_state
 *      authorises navigation and STOP is clear;
 *   5. publishes the IF-06 snapshot under its mutex.
 * It only reports. The Vehicle Controller decides and Motion Control alone
 * drives the motors (AGENTS.md).
 *
 * Barcode edges come from subsystem 2's shared GPIO dispatcher on GP26.
 * The interrupt only timestamps the new level into a single-producer,
 * single-consumer ring; DMB barriers order it for either core.
 */
#include "subsystem3.h"

#include "s3_config.h"
#include "s3_ir.h"
#include "s3_nav.h"
#include "s3_scan.h"
#include "s3_track.h"
#include "../subsystem2/drv_gpio_irq.h"
#include "../subsystem2/subsystem2.h"

#define S3_LINE_CORE        TP_PRC1     /* core 0, with the dispatcher */
#define S3_PUBLISH_TMO_MS   2           /* readers copy, never hold long */
#define S3_FRONT_LEFT_BIT   0x01U       /* s3_ir bit order */
#define S3_FRONT_RIGHT_BIT  0x02U
#define S3_BARCODE_BIT      0x04U

static ID               g_line_tid = 0;
static ID               g_line_cyc = 0;
static ID               g_line_mtx = 0;
static s3_line_status_t g_status;               /* guarded by g_line_mtx */

/* Edge ring: written by the interrupt, read by the task. */
static volatile uint32_t g_edge_us[S3_EDGE_RING_SIZE];
static volatile uint8_t  g_edge_level[S3_EDGE_RING_SIZE];
static volatile uint32_t g_edge_head = 0U;      /* interrupt only */
static volatile uint32_t g_edge_tail = 0U;      /* task only */
static volatile bool     gb_edge_lost = false;  /* set by interrupt */
static uint8_t           g_isr_level = 0U;      /* interrupt only */

static s3_symbol_t const g_alphabet[4] =
{
    {S3_BAR_PATTERN_A, 'A'}, {S3_BAR_PATTERN_B, 'B'},
    {S3_BAR_PATTERN_C, 'C'}, {S3_BAR_PATTERN_D, 'D'}
};

static void     line_task(INT stacd, void * p_exinf);
static void     line_cyclic(void * p_exinf);
static void     line_edge_isr(uint32_t gpio, uint32_t events, uint32_t now_us);
static void     line_input(uint32_t pin);
static void     line_barrier(void);
static uint8_t  line_barcode_level(void);
static uint32_t line_now_ms(void);
static bool     line_navigation_allowed(bool * p_stopped);

/** @brief Create the status mutex and the sampling clock. */
ER s3_line_create(void)
{
    T_CMTX cmtx = {0};
    T_CCYC ccyc = {0};

    cmtx.mtxatr = TA_INHERIT;

    ID const mtx = tk_cre_mtx(&cmtx);

    if (mtx < E_OK)
    {
        return mtx;
    }

    g_line_mtx = mtx;
    ccyc.cycatr = TA_HLNG;                      /* started with the task */
    ccyc.cychdr = (FP)line_cyclic;
    ccyc.cyctim = S3_PERIOD_MS;
    ccyc.cycphs = S3_PERIOD_MS;
    g_line_cyc = tk_cre_cyc(&ccyc);

    return (g_line_cyc < E_OK) ? g_line_cyc : E_OK;
}

/** @brief Configure the sensor inputs, then start the task and clock. */
ER s3_line_start(void)
{
    line_input(S3_PIN_FRONT_LEFT);
    line_input(S3_PIN_FRONT_RIGHT);
    line_input(S3_PIN_BARCODE);
    g_isr_level = line_barcode_level();

    ER const er = drv_gpio_irq_register(S3_PIN_BARCODE,
                                        DRV_GPIO_IRQ_EDGE_RISE
                                        | DRV_GPIO_IRQ_EDGE_FALL,
                                        line_edge_isr);

    if (er < E_OK)
    {
        return er;
    }

    ID const tid = app_start_task((FP)line_task, S3_PRI_LINE, S3_LINE_CORE,
                                  "line_sensor");

    if (tid < E_OK)
    {
        return tid;
    }

    g_line_tid = tid;

    return tk_sta_cyc(g_line_cyc);
}

ER s3_line_get(s3_line_status_t * p_status)
{
    ID const mtx = g_line_mtx;

    if (NULL == p_status)
    {
        return E_PAR;
    }

    if (mtx <= 0)
    {
        return E_OBJ;
    }

    ER const er = tk_loc_mtx(mtx, TMO_POL);

    if (er < E_OK)
    {
        return er;
    }

    *p_status = g_status;
    (void)tk_unl_mtx(mtx);

    return E_OK;
}

static void line_task(INT stacd, void * p_exinf)
{
    s3_calibration_t cal = {0};
    s3_track_cfg_t   track_cfg;
    s3_track_t       track;
    s3_scan_t        scan;
    s3_navigation_t  nav = {S3_NAV_NONE, 0U, 0U, false};
    s3_line_status_t st = {0};
    s3_scan_cfg_t const scan_cfg =
    {
        S3_BAR_RUN_MAX_US, S3_BAR_QUIET_US, S3_BAR_SENTINEL, g_alphabet, 4U
    };
    uint32_t scan_id = 0U;
    bool     b_was_stopped = true;

    (void)stacd;
    (void)p_exinf;

    s3_track_default_cfg(&track_cfg);

    /* Polarity is configured, not measured (S3-H01): a failed
     * calibration or track set-up would leave the sensors unusable, so
     * publish a latched fault instead of guessing. */
    bool const b_ready = s3_ir_calibrate(&cal,
                                         (uint8_t)(~S3_IR_BLACK_LEVELS
                                                   & (S3_IR_PATTERNS - 1U)),
                                         S3_IR_BLACK_LEVELS)
                         && s3_track_init(&track, &track_cfg, line_now_ms());

    s3_scan_init(&scan);

    for (;;)
    {
        uint8_t        black = 0U;
        s3_track_out_t out = {S3_LINE_UNKNOWN, 0, 0U, 0U, false, false, true};
        char           symbol = '\0';
        bool           b_stopped = true;

        (void)tk_slp_tsk(TMO_FEVR);

        INT const      missed = tk_can_wup(TSK_SELF);
        uint32_t const now_ms = line_now_ms();
        uint32_t const now_us = in_w(TIMER_TIMERAWL);

        if (missed > 0)
        {
            st.overruns = (uint8_t)(st.overruns + (uint8_t)missed);
        }

        /* 1. Barcode edges, oldest first. */
        while (g_edge_tail != g_edge_head)
        {
            uint32_t const idx = g_edge_tail;
            uint8_t        lvl;
            uint32_t       t_us;

            line_barrier();
            lvl  = g_edge_level[idx];
            t_us = g_edge_us[idx];
            line_barrier();
            g_edge_tail = (idx + 1U) % S3_EDGE_RING_SIZE;

            uint8_t norm = 0U;

            if (s3_ir_normalize(&cal, (uint8_t)(lvl << 2U), &norm)
                && (S3_SCAN_ERROR == s3_scan_edge(&scan, &scan_cfg,
                                                 (0U != (norm
                                                         & S3_BARCODE_BIT)),
                                                 t_us)))
            {
                st.barcode_errors = (uint8_t)(st.barcode_errors + 1U);
            }
        }

        if (gb_edge_lost)
        {
            gb_edge_lost = false;
            s3_scan_abort(&scan);           /* a hole in the edge record */
            st.barcode_errors = (uint8_t)(st.barcode_errors + 1U);
        }

        /* 2. Finish or expire a capture. */
        s3_scan_result_t const res = s3_scan_poll(&scan, &scan_cfg, now_us,
                                                  &symbol);

        if (S3_SCAN_DECODED == res)
        {
            scan_id++;
            st.barcodes    = (uint8_t)(st.barcodes + 1U);
            st.last_symbol = (uint8_t)symbol;
            (void)s3_nav_stage(&nav, symbol, scan_id, now_ms);
        }
        else if (S3_SCAN_ERROR == res)
        {
            st.barcode_errors = (uint8_t)(st.barcode_errors + 1U);
        }
        else
        {
            /* Nothing finished, or a single crossing bar. */
        }

        /* 3. Front sensors: one register read samples both at once. */
        UW const levels = in_w(GPIO_IN);
        uint8_t  raw = 0U;

        raw |= (0U != (levels & (1UL << S3_PIN_FRONT_LEFT)))
               ? S3_FRONT_LEFT_BIT : 0U;
        raw |= (0U != (levels & (1UL << S3_PIN_FRONT_RIGHT)))
               ? S3_FRONT_RIGHT_BIT : 0U;
        raw |= (0U != (levels & (1UL << S3_PIN_BARCODE)))
               ? S3_BARCODE_BIT : 0U;

        if (b_ready && s3_ir_normalize(&cal, raw, &black))
        {
            s3_track_update(&track, &track_cfg, black, now_ms, &out);
        }

        if (0U != (out.events & S3_TRACK_EV_JUNCTION))
        {
            st.junctions = (uint8_t)(st.junctions + 1U);
        }

        if (0U != (out.events & S3_TRACK_EV_LOST))
        {
            st.losses = (uint8_t)(st.losses + 1U);
        }

        /* 4. Navigation offer; STOP discards staged work and capture. */
        bool const b_auth = line_navigation_allowed(&b_stopped);

        if (b_stopped && (!b_was_stopped))
        {
            s3_scan_abort(&scan);
        }

        b_was_stopped = b_stopped;

        s3_command_t const cmd =
            s3_nav_take(&nav, b_stopped, b_auth,
                        (0U != (out.events & S3_TRACK_EV_JUNCTION)), now_ms,
                        S3_NAV_MAX_AGE_MS);

        if (S3_NAV_NONE != cmd)
        {
            st.nav_cmd = (uint8_t)cmd;
            st.nav_id  = nav.scan_id;
        }

        /* 5. Publish. */
        st.captured_ms    = now_ms;
        st.seq++;
        st.steer_permille = out.steer_permille;
        st.line           = (uint8_t)out.line;
        st.last_side      = out.last_side;
        st.flags          = (uint8_t)((out.valid ? S3_ST_VALID : 0U)
                            | (out.lost ? S3_ST_LOST : 0U)
                            | ((S3_LINE_JUNCTION == out.line)
                               ? S3_ST_JUNCTION : 0U)
                            | (out.fault ? S3_ST_FAULT : 0U)
                            | ((0U != (black & S3_TRACK_FRONT_BITS))
                               ? S3_ST_SEEN : 0U)
                            | ((0U != (black & S3_BARCODE_BIT))
                               ? S3_ST_BAR_BLACK : 0U));

        if (tk_loc_mtx(g_line_mtx, S3_PUBLISH_TMO_MS) >= E_OK)
        {
            g_status = st;
            (void)tk_unl_mtx(g_line_mtx);
        }
    }
}

static void line_cyclic(void * p_exinf)
{
    (void)p_exinf;

    if (g_line_tid > 0)
    {
        (void)tk_wup_tsk(g_line_tid);
    }
}

/* Dispatcher callback, interrupt context: one register read and a ring
 * store. Both edges in one service mean a pulse too short to time. */
static void line_edge_isr(uint32_t gpio, uint32_t events, uint32_t now_us)
{
    uint8_t const level = line_barcode_level();

    (void)gpio;

    if ((DRV_GPIO_IRQ_EDGE_RISE | DRV_GPIO_IRQ_EDGE_FALL)
        == (events & (DRV_GPIO_IRQ_EDGE_RISE | DRV_GPIO_IRQ_EDGE_FALL)))
    {
        gb_edge_lost = true;
    }

    if (level != g_isr_level)
    {
        uint32_t const head = g_edge_head;
        uint32_t const next = (head + 1U) % S3_EDGE_RING_SIZE;

        g_isr_level = level;

        if (next == g_edge_tail)
        {
            gb_edge_lost = true;            /* ring full */
        }
        else
        {
            g_edge_us[head]    = now_us;
            g_edge_level[head] = level;
            line_barrier();
            g_edge_head = next;
        }
    }
}

/* Inputs with the pad pull-up: the LM393 output is open collector, and a
 * disconnected module then reads black, which s3_track reports as FAULT. */
static void line_input(uint32_t pin)
{
    (void)gpio_set_pin(pin, GPIO_MODE_IN);
    clr_w(GPIO(pin), GPIO_PDE);
    set_w(GPIO(pin), GPIO_PUE);
}

/* Orders ring accesses as seen by the other core (ARMv6-M DMB). */
static void line_barrier(void)
{
    __asm volatile ("dmb" ::: "memory");
}

static uint8_t line_barcode_level(void)
{
    return (0U != (in_w(GPIO_IN) & (1UL << S3_PIN_BARCODE))) ? 1U : 0U;
}

static uint32_t line_now_ms(void)
{
    SYSTIM tim;

    return (tk_get_tim(&tim) >= E_OK) ? (uint32_t)tim.lo : 0U;
}

/* Navigation is authorised only by the controller's robot_state flag, and
 * never while STOP is latched. An unreadable flag counts as STOP. */
static bool line_navigation_allowed(bool * p_stopped)
{
    s2_robot_state_t rs;
    T_RFLG           rflg;
    bool const       b_auth = (E_OK == s2_get_robot_state(&rs))
                              && (0U != (rs.flags & S2_RS_NAV_AUTHORISED));

    *p_stopped = (tk_ref_flg(safety_flags, &rflg) < E_OK)
                 || (0U != (rflg.flgptn & SAFETY_FLG_STOP));

    return b_auth && (!*p_stopped);
}

/** End of file. */
