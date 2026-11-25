/*
 *       app_main.c
 *       START HERE.  usermain() below is your program's entry point.
 *
 *       How micro T-Kernel starts your code
 *       -----------------------------------
 *         reset -> kernel initialises itself -> creates the "initial task"
 *               -> initial task calls usermain()
 *
 *       usermain() runs inside that initial task at priority 1 (the highest).
 *       On a single core nothing you create runs until usermain() blocks, but
 *       on the dual-core build the OTHER core may start a new task at once.
 *       So usermain() works in two phases:
 *         1. every subsystem creates its kernel objects  (subsystemN_init)
 *         2. every subsystem starts its tasks             (subsystemN_start)
 *       and then puts the initial task to sleep forever.
 *       If usermain() RETURNS, the kernel shuts the whole system down.
 *
 *       The robot: one subsystem per folder (owners in TEAM.md)
 *       -------------------------------------------------------
 *         subsystem1/  WiFi, MQTT commands and telemetry; no task of its
 *                      own, it runs in the WiFi service task
 *         subsystem2/  Vehicle Controller and Motion Control tasks (core 1);
 *                      the only writer of motor PWM
 *         subsystem3/  line sensors and barcode reader; s3_follow_run() runs
 *                      in this initial task in the mission image
 *         subsystem4/  IMU task: LSM303DLHC on I2C1, humps, impacts, lifts
 *         subsystem5/  Obstacle Scan task: HC-SR04, servo, bypass plan
 *
 *       The interfaces between them (IF-05 onwards) are listed in TEAM.md
 *       section 5.  The subsystem that RECEIVES or OWNS the data creates the
 *       object and publishes its ID in its header; the other side just
 *       uses it.
 *
 *       Output appears on the USB serial port (115200 8N1 in any terminal).
 */

#include "app.h"
#include "app_stack.h"
#include "subsystem3/s3_follow.h"

#ifndef S3_HARDWARE_TEST
#define S3_HARDWARE_TEST 0
#endif

#ifndef APP_MISSION
#define APP_MISSION 0
#endif

#ifndef S2_CAL_TEST
#define S2_CAL_TEST 0
#endif

/*
 * Task stacks (review F-24). Each task gets a stack from this static pool,
 * painted before it starts, so app_stack_report() can show how much of it
 * was ever used. A pool stack is the size the kernel would allocate:
 * TASK_STACK_SIZE plus its DEFAULT_SYS_STKSZ (128 bytes on the Cortex-M0+),
 * rounded to 8 (the kernel's rule). The mission creates four tasks, five
 * with S4_RAW_LOG; if the pool runs out the kernel allocates as before and
 * that task is not reported.
 */
#define APP_STACK_TASKS     5u
#define APP_STACK_BYTES     ((((TASK_STACK_SIZE + DEFAULT_SYS_STKSZ) + 7u) \
                              / 8u) * 8u)

typedef struct
{
    const char * p_name;
    uint32_t *   p_words;   /* lowest address of the stack */
} app_stack_slot_t;

/* uint64_t elements give the 8-byte alignment AAPCS needs for a stack. */
static uint64_t         g_app_stack[APP_STACK_TASKS][APP_STACK_BYTES / 8u];
static app_stack_slot_t g_app_slot[APP_STACK_TASKS];
static uint32_t         g_app_slots_used = 0u;

/*
 * Create and start one task (declared in app.h for every subsystem).
 *   prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for "any core".
 */
EXPORT ID app_start_task (FP entry, PRI pri, UW prc, const char *p_name)
{
    T_CTSK  ctsk = {0};
    ID      tid;
    ER      ercd;

    ctsk.task    = entry;           /* function the task runs */
    ctsk.itskpri = pri;             /* initial priority (1 = highest) */
    ctsk.stksz   = TASK_STACK_SIZE; /* kernel allocates the stack */
    ctsk.tskatr  = TA_HLNG          /* written in a high-level language (C) */
                 | TA_RNG3;              /* runs at user protection level */
    if (g_app_slots_used < APP_STACK_TASKS)
    {
        /* A painted stack from the pool; the kernel takes all of it, so
           stksz is the whole buffer. The array is uint64_t only for
           alignment: viewing it as uint32_t words keeps every address in
           the same object. APP_STACK_BYTES (2176) fits INT. */
        uint32_t * const p_words =
            (uint32_t *)(void *)g_app_stack[g_app_slots_used];

        app_stack_paint(p_words, APP_STACK_BYTES / 4u);
        ctsk.tskatr |= TA_USERBUF;
        ctsk.bufptr  = p_words;
        ctsk.stksz   = (INT)APP_STACK_BYTES;
        g_app_slot[g_app_slots_used].p_name  = p_name;
        g_app_slot[g_app_slots_used].p_words = p_words;
        g_app_slots_used++;
    }
    else
    {
        tm_printf((UB*)"stack pool full: %s not reported\n", p_name);
    }
#if TK_SUPPORT_SMP
    if (0 != prc)
    {
        ctsk.tskatr |= TA_ASSPRC;       /* honour the affinity below */
        ctsk.assprc  = prc;             /* which core(s) it may run on */
    }
#else
    (void)prc;                      /* single-core build: no affinity */
#endif

    /* tk_cre_tsk: create the task.  It starts in the DORMANT state. */
    tid = tk_cre_tsk(&ctsk);
    if (tid < E_OK)
    {
        CHECK(tid, p_name);
        return tid;
    }

    /* tk_sta_tsk: DORMANT -> READY.  The int is passed as stacd. */
    ercd = tk_sta_tsk(tid, 0);
    CHECK(ercd, p_name);
    return tid;
}

EXPORT INT usermain (void)
{
    T_RVER  ver;

    /* tk_ref_ver: ask the kernel who it is */
    tk_ref_ver(&ver);
    tm_printf((UB*)"\n=== micro T-Kernel on Raspberry Pi Pico W"
              " (spec 0x%04x, product 0x%04x) ===\n", ver.spver, ver.prver);
#if TK_SUPPORT_SMP
    tm_printf((UB*)"dual-core (SMP) build, usermain on core %d\n",
              tk_get_prc() - 1);      /* tk_get_prc() counts from 1 */
#else
    tm_printf((UB*)"single-core build\n");
#endif


#if APP_MISSION
    /* The mission (normal image, and TEST=1 S3_LINE_TEST=1): Vehicle
       Controller and Motion Control (S2), IMU terrain (S4) and Obstacle
       Scan (S5) tasks; the line and barcode loop (S3) then runs here in
       the initial task. Subsystem 1 runs in the WiFi service task. */
    CHECK(subsystem2_init(), "subsystem2_init");
    CHECK(subsystem3_init(), "subsystem3_init");
    CHECK(subsystem4_init(), "subsystem4_init");
    CHECK(scan_lifecycle_init(), "scan_lifecycle_init");
    CHECK(subsystem2_start(), "subsystem2_start");
    CHECK(subsystem4_start(), "subsystem4_start");
    CHECK(scan_lifecycle_start(), "scan_lifecycle_start");
    tm_printf((UB *)"mission image: S2/S3/S4/S5 started\n");
    s3_follow_run();
#elif APP_TEST && S2_CAL_TEST
    /* Dedicated S2 calibration: GP20 runs one manoeuvre per press.
       S4 reports terrain alongside it (mode 1 drives the hump-height
       bench test, S4-H08); it does not affect motion. */
    CHECK(subsystem2_init(), "subsystem2_init");
    CHECK(subsystem4_init(), "subsystem4_init");
    CHECK(subsystem2_start(), "subsystem2_start");
    CHECK(subsystem4_start(), "subsystem4_start");
    tm_printf((UB *)"S2 calibration test mode %d\n", (INT)S2_CAL_TEST);
#elif APP_TEST && S3_HARDWARE_TEST
    /* Dedicated front-IR bench mode: no other application task runs. */
    CHECK(subsystem3_init(), "subsystem3_init");
    tm_printf((UB *)"S3 sensor-only test mode\n");
    subsystem3_test();
#else
    /* Self-test image (TEST=1 alone).
       Phase 1: every subsystem creates its kernel objects */
    CHECK(subsystem1_init(), "subsystem1_init");
    CHECK(subsystem2_init(), "subsystem2_init");
    CHECK(subsystem3_init(), "subsystem3_init");
    CHECK(subsystem4_init(), "subsystem4_init");
    CHECK(scan_lifecycle_init(), "scan_lifecycle_init");

    /* Phase 2: every subsystem starts its tasks */
    CHECK(subsystem1_start(), "subsystem1_start");
    CHECK(subsystem2_start(), "subsystem2_start");
    CHECK(subsystem3_start(), "subsystem3_start");
    CHECK(subsystem4_start(), "subsystem4_start");
    CHECK(scan_lifecycle_start(), "scan_lifecycle_start");

    tm_printf((UB*)"all subsystems started\n\n");

    /* Run every subsystem's tests in turn, here in the initial task,
       while the system is running.  Tests may call tk_dly_tsk() to let
       the other tasks work for a while. */
    tm_printf((UB*)"[TEST] === test run start ===\n");
    subsystem1_test();
    subsystem2_test();
    subsystem3_test();
    subsystem4_test();
    scan_lifecycle_test();
    tm_printf((UB*)"[TEST] === test run end ===\n");
#endif

    /* Sleep forever.  Returning from usermain() would shut down. */
    tk_slp_tsk(TMO_FEVR);
    return 0;
}

/*
 * Print the high-water mark of every pooled task stack: the bytes at the
 * bottom never written since the task was created (review F-24). The
 * scan reads at most 5 x 2176 bytes and writes nothing.
 */
EXPORT void app_stack_report (void)
{
    for (uint32_t idx = 0u; idx < g_app_slots_used; idx++)
    {
        uint32_t const unused = app_stack_unused_bytes(
            g_app_slot[idx].p_words, APP_STACK_BYTES / 4u);

        /* Byte counts are at most 2176, which UINT holds. */
        tm_printf((UB*)"stack %s: %u of %u bytes never used\n",
                  (UB*)g_app_slot[idx].p_name, (UINT)unused,
                  (UINT)APP_STACK_BYTES);
    }
}

/* End of file. */
