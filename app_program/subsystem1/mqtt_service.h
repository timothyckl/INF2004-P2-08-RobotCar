/** @file mqtt_service.h
 * @brief Subsystem 1 hooks called by the WiFi service task's MQTT client.
 *
 * The MQTT client (lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c) owns every
 * lwIP call and runs only in cyw43_utk_task. It calls these functions,
 * which own the single service_bridge instance (INTEGRATION.md sequence).
 * Every hook is bounded and never waits. Kernel-free on purpose: lwIP's and
 * the kernel's size_t definitions conflict (see s2_s1_adapter.h).
 *
 * Topics are S1's proposed values (PROPOSAL.md) until the team agrees
 * them; override them in config/wifi_secrets.h if needed.
 */
#ifndef MQTT_SERVICE_H
#define MQTT_SERVICE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef S1_MQTT_COMMAND_TOPIC
#define S1_MQTT_COMMAND_TOPIC   "p2-08/p2-08-car-01/command"
#endif

#ifndef S1_MQTT_TELEMETRY_TOPIC
#define S1_MQTT_TELEMETRY_TOPIC "p2-08/p2-08-car-01/telemetry"
#endif

#define S1_MQTT_TELEMETRY_MS    1000u   /* proposed period, unvalidated */
#define S1_MQTT_TELEMETRY_MAX   16u     /* encode buffer, >= codec size */

/**
 * @brief Configure the bridge (idempotent). Call before subscribing.
 * @return True when the bridge is usable.
 */
bool mqtt_service_init (void);

/**
 * @brief Topic callback: start staging one publication.
 * @param p_topic NUL-terminated topic from lwIP.
 * @param total_length Total payload length reported by lwIP.
 * @return True if it is the command topic and the payload is accepted.
 */
bool mqtt_service_topic (const char * p_topic, size_t total_length);

/**
 * @brief Data callback: pass one fragment; a complete command is routed
 *        to the Vehicle Controller (STOP by flag, START/RESET by queue).
 */
void mqtt_service_data (const void * p_data, size_t length, bool is_last);

/**
 * @brief Encode one telemetry snapshot.
 * @return Byte count to publish, or 0 to skip this period.
 */
size_t mqtt_service_telemetry (uint8_t * p_output, size_t capacity);

#endif /* MQTT_SERVICE_H */

/*** End of file. ***/
