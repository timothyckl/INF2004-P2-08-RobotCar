/*
 * subsystem1.h
 * Subsystem 1 -- WiFi/MQTT communication support. Student ID: 2501110.
 *
 *	This header is subsystem 1's PUBLIC interface: only what other
 *	subsystems are allowed to use.  Everything else stays LOCAL in the .c
 *	files.  Other subsystems include it as "../subsystem1/subsystem1.h".
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"

#define S1_PRI_LED 12	/* Temporary IF-05 compatibility task priority. */

/* IF-05: Retained temporarily so subsystem 4's starter monitor still links. */
IMPORT ID tid_led;

/* Private to subsystem 1, shared between its own .c files */
IMPORT void led_task(INT stacd, void * exinf);

#endif /* SUBSYSTEM1_H */
