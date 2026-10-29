/*
 * subsystem1.c
 * Subsystem 1 -- starts the temporary IF-05 compatibility task.
 */

#include "subsystem1.h"

EXPORT ID tid_led;	/* IF-05, see subsystem1.h. */

EXPORT ER subsystem1_init(void)
{
	return E_OK;
}

EXPORT ER subsystem1_start(void)
{
	tid_led = app_start_task(led_task, S1_PRI_LED, 0, "led_task");
	if (tid_led < E_OK)
	{
		return tid_led;
	}

	return E_OK;
}
