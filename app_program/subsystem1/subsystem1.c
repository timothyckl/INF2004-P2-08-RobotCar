/**
 * @file subsystem1.c
 * @brief Subsystem 1 lifecycle hooks.
 *
 * The MQTT service bridge runs inside the built-in WiFi task. The only S1
 * application task is a hardware-free compatibility sleeper retained until
 * subsystem 4 removes its starter IF-05 dependency.
 */
#include "subsystem1.h"

EXPORT ID tid_led;

EXPORT ER subsystem1_init (void)
{
    return (E_OK);
}

EXPORT ER subsystem1_start (void)
{
    tid_led = app_start_task(s1_compat_task, S1_PRI_LED, 0u,
                             "s1_compat_task");

    return ((tid_led < E_OK) ? tid_led : E_OK);
}

/* End of file. */
