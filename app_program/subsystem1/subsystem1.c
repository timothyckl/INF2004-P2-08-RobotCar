/*
 * subsystem1.c
 * Subsystem 1 -- WiFi/MQTT communication support.
 *
 * Subsystem 1 creates no application task or kernel object: its service
 * bridge runs inside the kernel's WiFi service task (mqtt_service.c, called
 * from lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c). The former IF-05
 * compatibility task was retired once no subsystem used tid_led.
 */

#include "subsystem1.h"

EXPORT ER subsystem1_init(void)
{
	return E_OK;
}

EXPORT ER subsystem1_start(void)
{
	return E_OK;
}
