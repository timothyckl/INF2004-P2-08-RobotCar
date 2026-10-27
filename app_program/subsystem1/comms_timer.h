/** @file comms_timer.h
 * @brief Nonblocking heartbeat and connection retry scheduling.
 */
#ifndef COMMS_TIMER_H
#define COMMS_TIMER_H

#include <stdbool.h>
#include <stdint.h>

typedef uint8_t comms_timer_action_t;

#define COMMS_TIMER_NONE       0u
#define COMMS_TIMER_WIFI       1u
#define COMMS_TIMER_MQTT       2u
#define COMMS_TIMER_HEARTBEAT  3u

typedef struct
{
    uint32_t heartbeat_ms;
    uint32_t retry_ms;
    uint32_t last_heartbeat_ms;
    uint32_t last_retry_ms;
    bool     heartbeat_armed;
    bool     retry_armed;
    bool     initialised;
} comms_timer_t;

/**
 * @brief Initialise a caller-owned scheduler with configurable periods.
 * @param p_timer Storage exclusively owned by one task, or externally locked.
 * @param heartbeat_ms Heartbeat period, from 1 through INT32_MAX ms.
 * @param retry_ms Retry period, from 1 through INT32_MAX ms.
 * @return True if initialised; false disables polling for non-NULL storage.
 */
bool comms_timer_init (comms_timer_t * p_timer, uint32_t heartbeat_ms,
                      uint32_t retry_ms);

/**
 * @brief Return one due action, recording its issue time without executing it.
 * @param p_timer Successfully initialised scheduler, exclusively owned.
 * @param now_ms Monotonic modulo-uint32 milliseconds, same clock throughout.
 * @param wifi_up True when WiFi is ready.
 * @param mqtt_up True when MQTT and command subscriptions are ready.
 * @param attempt_pending True while a connection attempt is outstanding.
 * @return A connection attempt, heartbeat, or NONE; never a movement command.
 * @note Poll at gaps no larger than INT32_MAX ms. No catch-up bursts occur.
 * The caller executes network actions only inside the WiFi service task.
 */
comms_timer_action_t comms_timer_poll (comms_timer_t * p_timer,
    uint32_t now_ms, bool wifi_up, bool mqtt_up, bool attempt_pending);

#endif /* COMMS_TIMER_H */

/*** End of file. ***/

