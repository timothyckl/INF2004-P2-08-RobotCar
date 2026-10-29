/** @file service_bridge.c
 * @brief MQTT fragmentation, validation, routing, and telemetry handoff.
 */
#include "service_bridge.h"

#include <string.h>

static void increment_saturated (uint32_t * p_value);
static service_bridge_result_t finish_command (
    service_bridge_t * p_bridge, uint32_t received_ms);
static service_bridge_result_t forward_command (
    service_bridge_t * p_bridge, command_parser_command_t command,
    uint32_t received_ms);
static service_bridge_result_t mark_invalid (service_bridge_t * p_bridge);
static bool port_is_valid (const service_bridge_port_t * p_port);

bool service_bridge_init (service_bridge_t * p_bridge,
    const char * p_command_topic, size_t topic_length,
    const service_bridge_port_t * p_port)
{
    bool valid = false;

    if (NULL != p_bridge)
    {
        *p_bridge = (service_bridge_t){0};

        if ((NULL != p_command_topic) && (topic_length > 0u) &&
            (topic_length < SERVICE_BRIDGE_TOPIC_MAX) &&
            port_is_valid(p_port))
        {
            memcpy(p_bridge->command_topic, p_command_topic, topic_length);
            p_bridge->command_topic[topic_length] = '\0';
            p_bridge->topic_length = topic_length;
            p_bridge->port = *p_port;
            p_bridge->initialised = true;
            valid = true;
        }
    }

    return valid;
}

bool service_bridge_begin (service_bridge_t * p_bridge,
    const char * p_topic, size_t topic_length, size_t total_length)
{
    bool accepted = false;

    if ((NULL != p_bridge) && p_bridge->initialised)
    {
        bool topic_matches = false;

        p_bridge->receiving = false;
        p_bridge->expected_length = 0u;
        p_bridge->received_length = 0u;

        if ((NULL != p_topic) && (topic_length == p_bridge->topic_length))
        {
            topic_matches = (0 == memcmp(p_topic,
                p_bridge->command_topic, topic_length));
        }

        if (!topic_matches)
        {
            increment_saturated(&p_bridge->stats.ignored_topics);
        }
        else if ((0u == total_length) ||
                 (total_length > COMMAND_PARSER_MAX))
        {
            increment_saturated(&p_bridge->stats.invalid_messages);
        }
        else
        {
            p_bridge->expected_length = total_length;
            p_bridge->receiving = true;
            accepted = true;
        }
    }

    return accepted;
}

service_bridge_result_t service_bridge_data (service_bridge_t * p_bridge,
    const void * p_data, size_t data_length, bool is_last,
    uint32_t received_ms)
{
    service_bridge_result_t result = SERVICE_BRIDGE_IGNORED;

    if ((NULL != p_bridge) && p_bridge->initialised && p_bridge->receiving)
    {
        const size_t remaining = p_bridge->expected_length -
                                 p_bridge->received_length;

        if (((NULL == p_data) && (data_length > 0u)) ||
            (data_length > remaining))
        {
            result = mark_invalid(p_bridge);
        }
        else
        {
            if (data_length > 0u)
            {
                memcpy(&p_bridge->command_data[p_bridge->received_length],
                       p_data, data_length);
                p_bridge->received_length += data_length;
            }

            if (is_last)
            {
                p_bridge->receiving = false;
                if (p_bridge->received_length == p_bridge->expected_length)
                {
                    result = finish_command(p_bridge, received_ms);
                }
                else
                {
                    result = mark_invalid(p_bridge);
                }
            }
            else if (p_bridge->received_length == p_bridge->expected_length)
            {
                result = mark_invalid(p_bridge);
            }
            else
            {
                result = SERVICE_BRIDGE_MORE;
            }
        }
    }

    return result;
}

size_t service_bridge_telemetry (service_bridge_t * p_bridge,
    uint8_t * p_output, size_t capacity)
{
    telemetry_encoder_snapshot_t snapshot = {0};
    size_t written = 0u;

    if ((NULL != p_bridge) && p_bridge->initialised &&
        (NULL != p_output) &&
        p_bridge->port.p_read_snapshot(p_bridge->port.p_context, &snapshot))
    {
        written = telemetry_encoder_encode(&snapshot, p_output, capacity);
    }

    return written;
}

bool service_bridge_get_stats (const service_bridge_t * p_bridge,
                              service_bridge_stats_t * p_stats)
{
    bool copied = false;

    if ((NULL != p_bridge) && p_bridge->initialised && (NULL != p_stats))
    {
        *p_stats = p_bridge->stats;
        copied = true;
    }

    return copied;
}

static void increment_saturated (uint32_t * p_value)
{
    if (UINT32_MAX != *p_value)
    {
        ++(*p_value);
    }
}

static service_bridge_result_t finish_command (
    service_bridge_t * p_bridge, uint32_t received_ms)
{
    const command_parser_command_t command = command_parser_parse(
        p_bridge->command_data, p_bridge->received_length);
    service_bridge_result_t result = SERVICE_BRIDGE_INVALID;

    if (COMMAND_PARSER_INVALID == command)
    {
        increment_saturated(&p_bridge->stats.invalid_messages);
    }
    else
    {
        result = forward_command(p_bridge, command, received_ms);
    }

    return result;
}

static service_bridge_result_t forward_command (
    service_bridge_t * p_bridge, command_parser_command_t command,
    uint32_t received_ms)
{
    bool delivered = false;
    service_bridge_result_t result = SERVICE_BRIDGE_UNDELIVERED;

    if (COMMAND_PARSER_STOP == command)
    {
        delivered = p_bridge->port.p_raise_remote_stop(
            p_bridge->port.p_context, received_ms);
    }
    else
    {
        delivered = p_bridge->port.p_send_regular_command(
            p_bridge->port.p_context, command, received_ms);
    }

    if (delivered)
    {
        increment_saturated(&p_bridge->stats.valid_commands);
        result = SERVICE_BRIDGE_FORWARDED;
    }
    else
    {
        increment_saturated(&p_bridge->stats.delivery_failures);
    }

    return result;
}

static service_bridge_result_t mark_invalid (service_bridge_t * p_bridge)
{
    p_bridge->receiving = false;
    increment_saturated(&p_bridge->stats.invalid_messages);

    return SERVICE_BRIDGE_INVALID;
}

static bool port_is_valid (const service_bridge_port_t * p_port)
{
    bool valid = false;

    if (NULL != p_port)
    {
        valid = (NULL != p_port->p_raise_remote_stop) &&
                (NULL != p_port->p_send_regular_command) &&
                (NULL != p_port->p_read_snapshot);
    }

    return valid;
}

/*** End of file. ***/

