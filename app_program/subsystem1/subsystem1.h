/**
 * @file subsystem1.h
 * @brief Subsystem 1 WiFi/MQTT communication support.
 */
#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"

/*
 * Network calls remain inside the kernel's built-in WiFi service task. The
 * task calls the narrow hooks declared by mqtt_service.h.
 */

/* Temporary IF-05 compatibility for subsystem 4's starter monitor. */
#define S1_PRI_LED 12

IMPORT ID tid_led;
IMPORT void s1_compat_task (INT start_code, void * p_exinf);

#endif /* SUBSYSTEM1_H */

/* End of file. */
