/** @file comms_test.c
 * @brief Hardware-independent regression tests using controlled inputs.
 */
#include "comms_test.h"

#if (defined(APP_TEST) && APP_TEST) || defined(S1_HOST_TEST)

#include "command_parser.h"
#include "comms_timer.h"
#include "telemetry_encoder.h"

#include <string.h>

static void check (bool condition, bool * p_passed);

bool comms_test_commands (void)
{
    static const char * const p_invalid[] =
    {
        "", "start", "Stop", "RESET\n", " STOP", "STOP ",
        "STAR", "STARTSTOP", "GO", "STOP\0junk"
    };
    static const size_t lengths[] = {0u, 5u, 4u, 6u, 5u, 5u,
                                    4u, 9u, 2u, 9u};
    static const uint8_t raw_stop[] = {'S', 'T', 'O', 'P'};
    static const uint8_t embedded_null[] = {'S', 'T', 0u, 'P'};
    bool passed = true;

    check((COMMAND_PARSER_START == command_parser_parse("START", 5u)),
              &passed);
    check((COMMAND_PARSER_RESET == command_parser_parse("RESET", 5u)), &passed);
    check((COMMAND_PARSER_STOP ==
               command_parser_parse(raw_stop, sizeof(raw_stop))), &passed);
    check((COMMAND_PARSER_INVALID == command_parser_parse(NULL, 4u)), &passed);
    check((COMMAND_PARSER_INVALID == command_parser_parse(NULL, 0u)), &passed);
    check((COMMAND_PARSER_INVALID == command_parser_parse("S", SIZE_MAX)),
              &passed);
    check((COMMAND_PARSER_INVALID ==
               command_parser_parse(embedded_null, sizeof(embedded_null))),
              &passed);
    check((COMMAND_PARSER_INVALID == command_parser_parse("STOP", 5u)),
              &passed);

    for (size_t idx = 0u; idx < (sizeof(lengths) / sizeof(lengths[0])); ++idx)
    {
        check((COMMAND_PARSER_INVALID ==
                   command_parser_parse(p_invalid[idx], lengths[idx])),
              &passed);
    }

    return passed;
}

bool comms_test_telemetry (void)
{
    telemetry_encoder_snapshot_t snapshot =
    {
        UINT32_C(0x12345678), UINT16_C(0xabcd), true, true, true
    };
    const uint8_t golden[] = {1u, 7u, 0x78u, 0x56u, 0x34u, 0x12u,
                              0xcdu, 0xabu};
    uint8_t output[TELEMETRY_ENCODER_SIZE + 1u] = {0u};
    bool    passed = true;

    output[TELEMETRY_ENCODER_SIZE] = 0xa5u;
    check((TELEMETRY_ENCODER_SIZE ==
               telemetry_encoder_encode(&snapshot, output, sizeof(output))),
              &passed);
    check((0 == memcmp(output, golden, sizeof(golden))), &passed);
    check((0xa5u == output[TELEMETRY_ENCODER_SIZE]), &passed);

    for (size_t capacity = 0u; capacity < TELEMETRY_ENCODER_SIZE; ++capacity)
    {
        memset(output, 0xa5, sizeof(output));
        check((0u ==
                   telemetry_encoder_encode(&snapshot, output, capacity)),
              &passed);

        for (size_t idx = 0u; idx < sizeof(output); ++idx)
        {
            check((0xa5u == output[idx]), &passed);
        }
    }

    check((0u == telemetry_encoder_encode(NULL, output, sizeof(output))),
              &passed);
    check((0xa5u == output[0]), &passed);
    check((0u == telemetry_encoder_encode(&snapshot, NULL,
                                               TELEMETRY_ENCODER_SIZE)),
              &passed);
    snapshot.captured_ms = UINT32_MAX;
    snapshot.distance_mm = UINT16_MAX;
    check((TELEMETRY_ENCODER_SIZE ==
               telemetry_encoder_encode(&snapshot, output, sizeof(output))),
              &passed);

    for (size_t idx = 2u; idx < TELEMETRY_ENCODER_SIZE; ++idx)
    {
        check((0xffu == output[idx]), &passed);
    }

    snapshot.distance_valid = false;
    snapshot.stopped        = false;
    snapshot.link_up        = false;
    snapshot.captured_ms    = 0u;
    check((TELEMETRY_ENCODER_SIZE ==
               telemetry_encoder_encode(&snapshot, output, sizeof(output))),
              &passed);

    for (size_t idx = 1u; idx < TELEMETRY_ENCODER_SIZE; ++idx)
    {
        check((0u == output[idx]), &passed);
    }

    snapshot.distance_valid = true;
    snapshot.distance_mm    = 0u;
    check((TELEMETRY_ENCODER_SIZE ==
               telemetry_encoder_encode(&snapshot, output, sizeof(output))),
              &passed);
    check((TELEMETRY_ENCODER_DISTANCE_VALID == output[1]), &passed);
    check((0u == output[6]), &passed);
    check((0u == output[7]), &passed);
    snapshot.distance_valid = false;
    snapshot.stopped        = true;
    check((TELEMETRY_ENCODER_SIZE ==
               telemetry_encoder_encode(&snapshot, output, sizeof(output))),
              &passed);
    check((TELEMETRY_ENCODER_STOPPED == output[1]), &passed);
    snapshot.stopped = false;
    snapshot.link_up = true;
    check((TELEMETRY_ENCODER_SIZE ==
               telemetry_encoder_encode(&snapshot, output, sizeof(output))),
              &passed);
    check((TELEMETRY_ENCODER_LINK_UP == output[1]), &passed);

    return passed;
}

bool comms_test_timers (void)
{
    comms_timer_t timer = {0};
    bool          passed = true;

    check(!comms_timer_init(NULL, 100u, 50u), &passed);
    check(!comms_timer_init(&timer, 0u, 50u), &passed);
    check(!comms_timer_init(&timer, 100u, 0u), &passed);
    check(!comms_timer_init(&timer, UINT32_MAX, 50u), &passed);
    check(!comms_timer_init(&timer, 100u, UINT32_MAX), &passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 0u, false, false, false)), &passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(NULL, 0u, true, true, false)), &passed);
    check(comms_timer_init(&timer, 100u, 50u), &passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 0u, false, false, true)), &passed);
    check((COMMS_TIMER_WIFI ==
               comms_timer_poll(&timer, 0u, false, false, false)), &passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 49u, false, false, false)), &passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 50u, false, false, true)), &passed);
    check((COMMS_TIMER_MQTT ==
               comms_timer_poll(&timer, 50u, true, false, false)), &passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 50u, true, false, false)), &passed);
    check((COMMS_TIMER_HEARTBEAT ==
               comms_timer_poll(&timer, 51u, true, true, false)), &passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 150u, true, true, false)), &passed);
    check((COMMS_TIMER_HEARTBEAT ==
               comms_timer_poll(&timer, 151u, true, true, false)), &passed);
    check((COMMS_TIMER_WIFI ==
               comms_timer_poll(&timer, 152u, false, false, false)), &passed);
    check((COMMS_TIMER_HEARTBEAT ==
               comms_timer_poll(&timer, 153u, true, true, false)), &passed);

    /* Start near clock wrap, then cross it for both independent schedules. */
    check(comms_timer_init(&timer, 100u, 50u), &passed);
    check((COMMS_TIMER_WIFI ==
        comms_timer_poll(&timer, UINT32_MAX - 24u, false, false, false)),
              &passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 24u, false, false, false)), &passed);
    check((COMMS_TIMER_WIFI ==
               comms_timer_poll(&timer, 25u, false, false, false)), &passed);
    check(comms_timer_init(&timer, 100u, 50u), &passed);
    check((COMMS_TIMER_HEARTBEAT ==
        comms_timer_poll(&timer, UINT32_MAX - 49u, true, true, false)),
              &passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 49u, true, true, false)), &passed);
    check((COMMS_TIMER_HEARTBEAT ==
               comms_timer_poll(&timer, 50u, true, true, false)), &passed);
    check((COMMS_TIMER_HEARTBEAT ==
               comms_timer_poll(&timer, 1000u, true, true, false)), &passed);
    check((COMMS_TIMER_NONE ==
               comms_timer_poll(&timer, 1000u, true, true, false)), &passed);
    check(comms_timer_init(&timer, UINT32_C(0x7fffffff), 1u), &passed);
    check((COMMS_TIMER_WIFI ==
               comms_timer_poll(&timer, 0u, false, true, false)), &passed);

    return passed;
}

/** @brief Accumulate a failed check without skipping subsequent cases. */
static void check (bool condition, bool * p_passed)
{
    if (!condition)
    {
        *p_passed = false;
    }
}

#endif

/*** End of file. ***/

