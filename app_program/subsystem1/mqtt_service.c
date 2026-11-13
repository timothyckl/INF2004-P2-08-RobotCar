/** @file mqtt_service.c
 * @brief Subsystem 1 service-task hooks (see mqtt_service.h).
 */
#include "mqtt_service.h"

#include <string.h>

#include "service_bridge.h"
#include "telemetry_encoder.h"
#include "../subsystem2/s2_s1_adapter.h"

static service_bridge_t g_bridge;       /* WiFi service task only */

bool mqtt_service_init (void)
{
    static const service_bridge_port_t port =
    {
        controller_raise_remote_stop,
        controller_send_remote_command,
        robot_read_telemetry,
        NULL
    };

    if (!g_bridge.initialised)
    {
        /* The adapters fail safely until subsystem2_init() has run, so a
           command arriving early is rejected, never queued for later. */
        (void) service_bridge_init(&g_bridge, S1_MQTT_COMMAND_TOPIC,
                                   sizeof(S1_MQTT_COMMAND_TOPIC) - 1u,
                                   &port);
    }

    return g_bridge.initialised;
}

bool mqtt_service_topic (const char * p_topic, size_t total_length)
{
    bool accepted = false;

    if ((NULL != p_topic) && g_bridge.initialised)
    {
        accepted = service_bridge_begin(&g_bridge, p_topic,
                                        strlen(p_topic), total_length);
    }

    return accepted;
}

void mqtt_service_data (const void * p_data, size_t length, bool is_last,
                        uint32_t received_ms)
{
    if (g_bridge.initialised)
    {
        (void) service_bridge_data(&g_bridge, p_data, length, is_last,
                                   received_ms);
    }
}

size_t mqtt_service_telemetry (uint8_t * p_output, size_t capacity)
{
    size_t length = 0u;

    if (g_bridge.initialised)
    {
        length = service_bridge_telemetry(&g_bridge, p_output, capacity);
    }

    return length;
}

/*** End of file. ***/

