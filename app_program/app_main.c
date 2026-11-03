/*
 *	app_main.c
 *	START HERE.  usermain() below is your program's entry point.
 *
 *	How micro T-Kernel starts your code
 *	-----------------------------------
 *	  reset -> kernel initialises itself -> creates the "initial task"
 *	        -> initial task calls usermain()
 *
 *	usermain() runs inside that initial task at priority 1 (the highest).
 *	On a single core nothing you create runs until usermain() blocks, but
 *	on the dual-core build the OTHER core may start a new task at once.
 *	So usermain() works in two phases:
 *	  1. every subsystem creates its kernel objects  (subsystemN_init)
 *	  2. every subsystem starts its tasks             (subsystemN_start)
 *	and then puts the initial task to sleep forever.
 *	If usermain() RETURNS, the kernel shuts the whole system down.
 *
 *	The robot: one subsystem per folder (owners in TEAM.md)
 *	--------------------------------------------------------
 *	  subsystem1/  WiFi/MQTT bridge: runs inside the kernel WiFi task
 *	               (mqtt_service.c via mqtt_utk.c); no task of its own
 *	  subsystem2/  Vehicle Controller (14, core 1) and Motion Control
 *	               (15, core 1): mission states, STOP latch, motors,
 *	               encoders, watchdog; creates the shared GPIO dispatcher
 *	  subsystem3/  Line Sensor task (16, core 0): IR line + barcode
 *	  subsystem4/  IMU task (18): terrain and highest hump
 *	  subsystem5/  Obstacle Scan task (20): HC-SR04, servo, bypass plan
 *
 *	How they talk to each other (TEAM.md section 5):
 *	  S1 -> S2  controller_flags REMOTE_STOP, remote_q START/RESET
 *	  S2 -> S1  robot_state snapshot for telemetry
 *	  S3 -> S2  IF-06: s3_line_get() snapshot (steering, line, nav_cmd)
 *	  S2 -> S3  robot_state NAV_AUTHORISED, safety_flags STOP
 *	  S2 -> S5  scan_q (REQUEST, AUTHORISE, STOP_CONFIRM, CANCEL, STOP)
 *	  S5 -> S2  controller_flags HAZARD/SCAN_FAULT, scan_task_get() plan
 *	  S2 -> S4  motion_get_status() distance for hump height
 *	The subsystem that RECEIVES or OWNS the data creates the object and
 *	publishes its ID or accessor in its header; the other side just uses
 *	it.
 *
 *	Output appears on the USB serial port (115200 8N1 in any terminal).
 */

#include "app.h"

/*
 * Create and start one task (declared in app.h for every subsystem).
 *   prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for "any core".
 */
EXPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name)
{
	T_CTSK	ctsk = {0};
	ID	tid;
	ER	er;

	ctsk.task    = entry;		/* function the task runs */
	ctsk.itskpri = pri;		/* initial priority (1 = highest) */
	ctsk.stksz   = TASK_STACK_SIZE;	/* kernel allocates the stack */
	ctsk.tskatr  = TA_HLNG		/* written in a high-level language (C) */
		     | TA_RNG3;		/* runs at user protection level */
#if TK_SUPPORT_SMP
	if (prc != 0) {
		ctsk.tskatr |= TA_ASSPRC;	/* honour the affinity below */
		ctsk.assprc  = prc;		/* which core(s) it may run on */
	}
#else
	(void)prc;			/* single-core build: no affinity */
#endif

	/* tk_cre_tsk: create the task.  It starts in the DORMANT state. */
	tid = tk_cre_tsk(&ctsk);
	if (tid < E_OK) {
		CHECK(tid, name);
		return tid;
	}

	/* tk_sta_tsk: DORMANT -> READY.  The int is passed as stacd. */
	er = tk_sta_tsk(tid, 0);
	CHECK(er, name);
	return tid;
}

EXPORT INT usermain(void)
{
	T_RVER	ver;

	/* tk_ref_ver: ask the kernel who it is */
	tk_ref_ver(&ver);
	tm_printf((UB*)"\n=== micro T-Kernel on Raspberry Pi Pico W"
		  " (spec 0x%04x, product 0x%04x) ===\n", ver.spver, ver.prver);
#if TK_SUPPORT_SMP
	tm_printf((UB*)"dual-core (SMP) build, usermain on core %d\n",
		  tk_get_prc() - 1);	/* tk_get_prc() counts from 1 */
#else
	tm_printf((UB*)"single-core build\n");
#endif

	/* Phase 1: every subsystem creates its kernel objects */
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

#if APP_TEST
	/* Test build (make TEST=1): run every subsystem's tests in turn,
	   here in the initial task, while the system is running.  Tests may
	   call tk_dly_tsk() to let the other tasks work for a while. */
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
