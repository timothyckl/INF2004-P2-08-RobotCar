/*
 * task_led.c
 * Subsystem 1 -- temporary compatibility task for starter interface IF-05.
 *
 * The old starter task drove GP19 as a blinking LED. Subsystem 1 no longer
 * owns that GPIO, so this task performs no GPIO access. It remains only until
 * subsystem 4 removes its starter suspend/resume calls for tid_led.
 */

#include "subsystem1.h"

EXPORT void led_task(INT stacd, void * exinf)
{
	(void) stacd;
	(void) exinf;

	while (1)
	{
		(void) tk_slp_tsk(TMO_FEVR);
	}
}
