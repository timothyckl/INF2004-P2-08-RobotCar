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

/* No public objects: commands and telemetry pass through the WiFi service
   task (mqtt_service.h) and subsystem 2's adapters (s2_s1_adapter.h). */

#endif /* SUBSYSTEM1_H */
