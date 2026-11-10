/*
 *	subsystem2.c
 *	Subsystem 2 -- Vehicle Controller + Motion Control: creates the
 *	objects and starts the tasks.
 *
 *	subsystem2_init():  controller objects (flags, remote_q,
 *	    state_mutex), then motion objects (motion_q, status mutex,
 *	    10 ms cyclic handler), then the shared GPIO dispatcher on this
 *	    core (core 0, usermain), so every subsystem can register pins
 *	    in its own _start().  Then the LEGACY starter objects.
 *	subsystem2_start(): Motion Control, then the Vehicle Controller.
 *	    Neither runs in a bench build (make S2_HW_TEST=n):
 *	    the bench sequence owns the motors.  Then the LEGACY producer.
 *
 *	The starter producer (IF-01, IF-02) stays until subsystem 3's
 *	consumer stops using it (docs/subsystem2.md, "Removing the
 *	starter").
 */

#include "subsystem2.h"
#include "drv_gpio_irq.h"
#include "drv_watchdog.h"
#include "task_controller.h"
#include "task_motion.h"

#ifndef S3_LINE_FOLLOW_TEST
#define S3_LINE_FOLLOW_TEST 0
#endif

#ifndef S2_CAL_TEST
#define S2_CAL_TEST 0
#endif

/* Dedicated trials that do not start the legacy producer. */
#define S2_NO_PRODUCER      (S3_LINE_FOLLOW_TEST || S2_CAL_TEST)

/* Robot interface objects (subsystem2.h). Set by s2_controller_create()
   once each object exists; 0 before that, so the S1 adapters (which may
   run first, in the WiFi task) fail safely. */
EXPORT ID	controller_flags = 0;
EXPORT ID	safety_flags = 0;
EXPORT ID	remote_q = 0;
EXPORT ID	line_q = 0;

EXPORT ID	sem_slots;	/* IF-02, see subsystem2.h */
EXPORT ID	mpf_samples;

EXPORT ER subsystem2_init(void)
{
	T_CSEM	csem = {0};
	T_CMPF	cmpf = {0};
	ER	er;

	/* Evidence that the watchdog protection fired (S2-H05) */
	if (drv_watchdog_caused_reset()) {
		tm_printf((UB*)"[S2] previous reset: WATCHDOG\n");
	}

	er = s2_controller_create();
	if (er < E_OK) {
		return er;
	}
	er = s2_motion_create();
	if (er < E_OK) {
		return er;
	}
	er = drv_gpio_irq_init();
	if (er < E_OK) {
		return er;
	}

	/* LEGACY starter objects (IF-01, IF-02) */

	/* Semaphore: a counter of free "slots".  The producer takes one
	   (tk_wai_sem) before each sample; the consumer gives one back
	   (tk_sig_sem).  It never exceeds the number of memory blocks, so
	   the producer can never run out of memory. */
	csem.sematr  = TA_TFIFO;	/* waiting tasks queue in FIFO order */
	csem.isemcnt = S2_POOL_COUNT;	/* initial count */
	csem.maxsem  = S2_POOL_COUNT;	/* maximum count */
	sem_slots = tk_cre_sem(&csem);
	if (sem_slots < E_OK) {
		return sem_slots;
	}

	/* Fixed-size memory pool: S2_POOL_COUNT blocks, each exactly one
	   SAMPLE_MSG.  Faster and more predictable than malloc(). */
	cmpf.mpfatr = TA_TFIFO | TA_RNG3;
	cmpf.mpfcnt = S2_POOL_COUNT;		/* number of blocks */
	cmpf.blfsz  = sizeof(SAMPLE_MSG);	/* size of one block */
	mpf_samples = tk_cre_mpf(&cmpf);
	return (mpf_samples < E_OK) ? mpf_samples : E_OK;
}

EXPORT ER subsystem2_start(void)
{
#if !S2_NO_PRODUCER
	ID	tid;
#endif

#if !S2_HW_TEST
	ER	er;

	er = s2_motion_start();
	if (er < E_OK) {
		return er;
	}
	er = s2_controller_start();
	if (er < E_OK) {
		return er;
	}
#endif

#if S2_NO_PRODUCER
	/* The line-follow and calibration trials do not use the legacy producer. */
	return E_OK;
#else
	/* LEGACY producer, pinned to core 1: samples cross to subsystem 3 */
	tid = app_start_task(producer_task, S2_PRI_PRODUCER, TP_PRC2,
			     "producer_task");
	return (tid < E_OK) ? tid : E_OK;
#endif
}
