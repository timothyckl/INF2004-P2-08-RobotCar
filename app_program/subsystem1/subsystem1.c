/**
 * @file subsystem1.c
 * @brief Subsystem 1 lifecycle hooks.
 *
 * Subsystem 1 creates no application task or kernel object: its MQTT
 * service bridge runs inside the built-in WiFi task (mqtt_service.c,
 * called from lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c). The IF-05
 * LED task was retired because no subsystem uses tid_led any more.
 */
#include "subsystem1.h"

EXPORT ER subsystem1_init (void)
{
    return (E_OK);
}

EXPORT ER subsystem1_start (void)
{
    return (E_OK);
}

/* End of file. */
