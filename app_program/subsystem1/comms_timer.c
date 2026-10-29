/** @file comms_timer.c
 * @brief Caller-driven scheduling with unsigned wraparound-safe elapsed times.
 */
#include "comms_timer.h"

#include <stddef.h>

static comms_timer_action_t poll_offline (comms_timer_t * p_timer,
    uint32_t now_ms, bool wifi_up, bool attempt_pending);
static comms_timer_action_t poll_online (comms_timer_t * p_timer,
                                        uint32_t now_ms);

bool comms_timer_init (comms_timer_t * p_timer, uint32_t heartbeat_ms,
                      uint32_t retry_ms)
{
    bool valid = false;

    if (NULL != p_timer)
    {
        *p_timer = (comms_timer_t){0};
        valid = (heartbeat_ms > 0u) &&
                (heartbeat_ms <= UINT32_C(0x7fffffff)) &&
                (retry_ms > 0u) && (retry_ms <= UINT32_C(0x7fffffff));
        p_timer->heartbeat_ms = heartbeat_ms;
        p_timer->retry_ms     = retry_ms;
        p_timer->initialised  = valid;
    }

    return valid;
}

comms_timer_action_t comms_timer_poll (comms_timer_t * p_timer,
    uint32_t now_ms, bool wifi_up, bool mqtt_up, bool attempt_pending)
{
    comms_timer_action_t action = COMMS_TIMER_NONE;

    if ((NULL != p_timer) && p_timer->initialised)
    {
        if (wifi_up && mqtt_up)
        {
            action = poll_online(p_timer, now_ms);
        }
        else
        {
            action = poll_offline(p_timer, now_ms, wifi_up, attempt_pending);
        }
    }

    return action;
}

static comms_timer_action_t poll_offline (comms_timer_t * p_timer,
    uint32_t now_ms, bool wifi_up, bool attempt_pending)
{
    comms_timer_action_t action = COMMS_TIMER_NONE;

    p_timer->heartbeat_armed = false;

    /* The cast preserves modulo-uint32 elapsed time after wrapping. */
    if ((!attempt_pending) &&
        ((!p_timer->retry_armed) ||
         ((uint32_t)(now_ms - p_timer->last_retry_ms) >=
          p_timer->retry_ms)))
    {
        p_timer->last_retry_ms = now_ms;
        p_timer->retry_armed   = true;
        action = wifi_up ? COMMS_TIMER_MQTT : COMMS_TIMER_WIFI;
    }

    return action;
}

static comms_timer_action_t poll_online (comms_timer_t * p_timer,
                                        uint32_t now_ms)
{
    comms_timer_action_t action = COMMS_TIMER_NONE;

    p_timer->retry_armed = false;

    /* The cast preserves modulo-uint32 elapsed time after wrapping. */
    if ((!p_timer->heartbeat_armed) ||
        ((uint32_t)(now_ms - p_timer->last_heartbeat_ms) >=
         p_timer->heartbeat_ms))
    {
        p_timer->last_heartbeat_ms = now_ms;
        p_timer->heartbeat_armed   = true;
        action = COMMS_TIMER_HEARTBEAT;
    }

    return action;
}

/*** End of file. ***/

