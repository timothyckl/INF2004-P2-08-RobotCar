/** @file service_bridge.h
 * @brief Boundary between the MQTT service task and application providers.
 */
#ifndef SERVICE_BRIDGE_H
#define SERVICE_BRIDGE_H

#include "command_parser.h"
#include "telemetry_encoder.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SERVICE_BRIDGE_TOPIC_MAX 64u

typedef uint8_t service_bridge_result_t;

#define SERVICE_BRIDGE_IGNORED       0u
#define SERVICE_BRIDGE_MORE          1u
#define SERVICE_BRIDGE_FORWARDED     2u
#define SERVICE_BRIDGE_INVALID       3u
#define SERVICE_BRIDGE_UNDELIVERED   4u

typedef bool (*service_bridge_stop_fn_t)(void * p_context,
                                         uint32_t received_ms);

typedef bool (*service_bridge_command_fn_t)(
    void * p_context, command_parser_command_t command, uint32_t received_ms);

typedef bool (*service_bridge_snapshot_fn_t)(
    void * p_context, telemetry_encoder_snapshot_t * p_snapshot);

typedef struct
{
    service_bridge_stop_fn_t     p_raise_remote_stop;
    service_bridge_command_fn_t  p_send_regular_command;
    service_bridge_snapshot_fn_t p_read_snapshot;
    void *                       p_context;
} service_bridge_port_t;

typedef struct
{
    uint32_t valid_commands;
    uint32_t invalid_messages;
    uint32_t delivery_failures;
    uint32_t ignored_topics;
} service_bridge_stats_t;

typedef struct
{
    service_bridge_port_t  port;
    service_bridge_stats_t stats;
    char                   command_topic[SERVICE_BRIDGE_TOPIC_MAX];
    uint8_t                command_data[COMMAND_PARSER_MAX];
    size_t                 topic_length;
    size_t                 expected_length;
    size_t                 received_length;
    bool                   receiving;
    bool                   initialised;
} service_bridge_t;

/**
 * @brief Configure one service-task-owned bridge and copy its topic and port.
 * @param p_bridge Storage used only by the single WiFi service task.
 * @param p_command_topic Exact command topic bytes, without a terminator.
 * @param topic_length Topic length from 1 through 63 bytes.
 * @param p_port Controller and telemetry functions; all are required.
 * @return True only when the complete configuration is usable.
 * @note The port context must remain valid for the bridge lifetime.
 */
bool service_bridge_init (service_bridge_t * p_bridge,
    const char * p_command_topic, size_t topic_length,
    const service_bridge_port_t * p_port);

/**
 * @brief Start staging one MQTT publication after its topic callback.
 * @param p_bridge An initialised service-task-owned bridge.
 * @param p_topic Exact received topic bytes.
 * @param topic_length Received topic length.
 * @param total_length Total MQTT payload length reported by lwIP.
 * @return True if this exact command topic and bounded payload are accepted.
 * @note Every call abandons any incomplete prior publication.
 */
bool service_bridge_begin (service_bridge_t * p_bridge,
    const char * p_topic, size_t topic_length, size_t total_length);

/**
 * @brief Append an MQTT payload fragment and dispatch a complete command.
 * @param p_bridge Bridge previously accepted by service_bridge_begin().
 * @param p_data Fragment bytes; may be NULL only when data_length is zero.
 * @param data_length Fragment byte count.
 * @param is_last True only for MQTT_DATA_FLAG_LAST.
 * @param received_ms Local receipt time supplied by the service adapter.
 * @return MORE, FORWARDED, INVALID, UNDELIVERED, or IGNORED.
 * @note STOP uses only p_raise_remote_stop. START and RESET use only the
 * regular-command function. No callback authorises motion by itself.
 */
service_bridge_result_t service_bridge_data (service_bridge_t * p_bridge,
    const void * p_data, size_t data_length, bool is_last,
    uint32_t received_ms);

/**
 * @brief Read a consistent provider snapshot and encode provisional telemetry.
 * @return Encoded byte count, or zero on provider/argument/capacity failure.
 */
size_t service_bridge_telemetry (service_bridge_t * p_bridge,
    uint8_t * p_output, size_t capacity);

/** @brief Copy service-task-owned counters for diagnostics or telemetry. */
bool service_bridge_get_stats (const service_bridge_t * p_bridge,
                              service_bridge_stats_t * p_stats);

#endif /* SERVICE_BRIDGE_H */

/*** End of file. ***/

