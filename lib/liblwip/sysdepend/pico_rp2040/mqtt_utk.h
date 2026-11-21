/**
 * @file mqtt_utk.h
 * @brief MQTT client wiring for micro T-Kernel on RP2040.
 */
#ifndef MQTT_UTK_H
#define MQTT_UTK_H

/*
 * Called by cyw43_utk_task() while the WiFi link is up. Connects to the
 * configured broker, subscribes to the command topic, and publishes heartbeat
 * and telemetry messages. Repeated calls while connecting or connected are
 * safe no-ops.
 */
void mqtt_utk_start (void);

#endif /* MQTT_UTK_H */

/* End of file. */
