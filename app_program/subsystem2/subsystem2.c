/*
 *	subsystem2.c
 *	Subsystem 2 -- Vehicle Controller + Motion Control: creates the
 *	objects and starts the tasks.
 *
 *	subsystem2_init():  controller objects (flags, remote_q,
 *	    state_mutex), then motion objects (motion_q, status mutex,
 *	    10 ms cyclic handler), then the shared GPIO dispatcher on this
 *	    core (core 0, usermain), so every subsystem can register pins
 *	    in its own _start().
 *	subsystem2_start(): Motion Control, then the Vehicle Controller.
 *	    Neither runs in a bench build (S2_HW_TEST in task_motion.h):
 *	    the bench sequence owns the motors.
 *
 *	The starter producer (IF-01, IF-02) was retired once subsystem 3's
 *	consumer was replaced by its Line Sensor task.
 */

#include "subsystem2.h"
#include "drv_gpio_irq.h"
#include "drv_watchdog.h"
#include "task_controller.h"
#include "task_motion.h"

/* Robot interface objects (subsystem2.h). Set by s2_controller_create()
   once each object exists; 0 before that, so the S1 adapters (which may
   run first, in the WiFi task) fail safely. */
EXPORT ID	controller_flags = 0;
EXPORT ID	safety_flags = 0;
EXPORT ID	remote_q = 0;
EXPORT ID	scan_q = 0;	/* controller to obstacle scan */

EXPORT ER subsystem2_init(void)
{
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
	return drv_gpio_irq_init();
}

EXPORT ER subsystem2_start(void)
{
#if !S2_HW_TEST
	ER	er;

	er = s2_motion_start();
	if (er < E_OK) {
		return er;
	}
	return s2_controller_start();
#else
	return E_OK;
#endif
}
