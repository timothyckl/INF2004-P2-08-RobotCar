/**
 * @file mqtt_utk.c
 * @brief MQTT client wiring for micro T-Kernel on RP2040.
 *
 * Every lwIP and MQTT call in this file runs in cyw43_utk_task(), the
 * processor-1 task that owns the CYW43439 and the NO_SYS lwIP stack.
 */
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/sys.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include "../../../../app_program/subsystem1/mqtt_service.h"

#include <tm/tmonitor.h>

#define MQTT_UTK_HEARTBEAT_MS 5000u
#define MQTT_UTK_KEEPALIVE_S  60u
#define MQTT_UTK_RETRY_MS     3000u

static mqtt_client_t * g_p_client;
static ip_addr_t       g_broker_addr;
static bool            g_connecting;

static void arm_retry (void);
static void heartbeat_timeout (void * p_arg);
static void telemetry_timeout (void * p_arg);

static void incoming_publish_cb (void * p_arg, const char * p_topic,
                                 u32_t total_length)
{
    (void) p_arg;
    (void) mqtt_service_topic(p_topic, (size_t) total_length);
}

static void incoming_data_cb (void * p_arg, const u8_t * p_data,
                              u16_t length, u8_t flags)
{
    bool const b_last = (flags & MQTT_DATA_FLAG_LAST) != 0u;

    (void) p_arg;
    mqtt_service_data(p_data, (size_t) length, b_last, sys_now());
}

static void subscribe_done_cb (void * p_arg, err_t result)
{
    (void) p_arg;

    if (ERR_OK == result)
    {
        tm_printf((UB *) "mqtt: subscribed to "
                  S1_MQTT_COMMAND_TOPIC "\n");
    }
    else
    {
        tm_printf((UB *) "mqtt: subscribe failed, err=%d\n", (int) result);
    }
}

static void publish_done_cb (void * p_arg, err_t result)
{
    (void) p_arg;

    if (ERR_OK != result)
    {
        tm_printf((UB *) "mqtt: publish failed, err=%d\n", (int) result);
    }
}

static void heartbeat_timeout (void * p_arg)
{
    static unsigned int count;
    UB                  payload[48];
    int                 length;

    (void) p_arg;

    if ((NULL != g_p_client) && mqtt_client_is_connected(g_p_client))
    {
        length = tm_sprintf(payload, (const UB *) "hb %u", count);
        count++;
        tm_printf((UB *) "mqtt: TX [" MQTT_TOPIC "] \"%s\"\n", payload);
        (void) mqtt_publish(g_p_client, MQTT_TOPIC, payload, (u16_t) length,
                            0u, 0u, publish_done_cb, NULL);
    }

    sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
}

static void telemetry_timeout (void * p_arg)
{
    u8_t   payload[S1_MQTT_TELEMETRY_MAX];
    size_t length;

    (void) p_arg;

    if ((NULL != g_p_client) && mqtt_client_is_connected(g_p_client))
    {
        length = mqtt_service_telemetry(payload, sizeof(payload));

        if (length > 0u)
        {
            (void) mqtt_publish(g_p_client, S1_MQTT_TELEMETRY_TOPIC,
                                payload, (u16_t) length, 0u, 0u,
                                publish_done_cb, NULL);
        }
    }

    sys_timeout(S1_MQTT_TELEMETRY_MS, telemetry_timeout, NULL);
}

static void retry_timeout (void * p_arg)
{
    (void) p_arg;
    g_connecting = false;
    mqtt_utk_start();
}

static void arm_retry (void)
{
    g_connecting = true;
    sys_untimeout(retry_timeout, NULL);
    sys_timeout(MQTT_UTK_RETRY_MS, retry_timeout, NULL);
}

static void connection_cb (mqtt_client_t * p_client, void * p_arg,
                           mqtt_connection_status_t status)
{
    (void) p_client;
    (void) p_arg;

    if (MQTT_CONNECT_ACCEPTED == status)
    {
        err_t result;

        g_connecting = false;
        sys_untimeout(retry_timeout, NULL);
        tm_printf((UB *) "mqtt: connected to " MQTT_BROKER_IP "\n");

        if (mqtt_service_init())
        {
            mqtt_set_inpub_callback(g_p_client, incoming_publish_cb,
                                    incoming_data_cb, NULL);
            result = mqtt_subscribe(g_p_client, S1_MQTT_COMMAND_TOPIC, 1u,
                                    subscribe_done_cb, NULL);

            if (ERR_OK != result)
            {
                tm_printf((UB *) "mqtt: mqtt_subscribe() failed, err=%d\n",
                          (int) result);
            }
        }
        else
        {
            tm_printf((UB *) "mqtt: subsystem 1 bridge init failed\n");
        }

        /*
         * Remove timeout chains from an earlier connection before starting
         * one heartbeat chain and one telemetry chain.
         */
        sys_untimeout(heartbeat_timeout, NULL);
        sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
        sys_untimeout(telemetry_timeout, NULL);
        sys_timeout(S1_MQTT_TELEMETRY_MS, telemetry_timeout, NULL);
    }
    else
    {
        tm_printf((UB *) "mqtt: connection status %d, retrying in %us\n",
                  (int) status, MQTT_UTK_RETRY_MS / 1000u);
        arm_retry();
    }
}

void mqtt_utk_start (void)
{
    struct mqtt_connect_client_info_t client_info = {0};
    err_t                             result;

    if (g_connecting
        || ((NULL != g_p_client) && mqtt_client_is_connected(g_p_client)))
    {
        return;
    }

    if (NULL == g_p_client)
    {
        g_p_client = mqtt_client_new();

        if (NULL == g_p_client)
        {
            tm_printf((UB *) "mqtt: mqtt_client_new() failed\n");
            arm_retry();
            return;
        }
    }

    if (!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&g_broker_addr)))
    {
        tm_printf((UB *) "mqtt: bad MQTT_BROKER_IP\n");
        arm_retry();
        return;
    }

    IP_SET_TYPE(&g_broker_addr, IPADDR_TYPE_V4);
    client_info.client_id  = MQTT_CLIENT_ID;
    client_info.keep_alive = MQTT_UTK_KEEPALIVE_S;
    g_connecting = true;
    result = mqtt_client_connect(g_p_client, &g_broker_addr,
                                 MQTT_BROKER_PORT, connection_cb, NULL,
                                 &client_info);

    if (ERR_OK != result)
    {
        tm_printf((UB *) "mqtt: mqtt_client_connect() failed, err=%d\n",
                  (int) result);
        arm_retry();
    }
}

/* End of file. */
