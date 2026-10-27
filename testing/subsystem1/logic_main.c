/** @file logic_main.c
 * @brief Host runner for the same logic tests available to Pico W firmware.
 */
#include "logic_main.h"

#include "../../app_program/subsystem1/comms_test.h"

#include <stdio.h>

/** @brief Print each test group's result and return nonzero on failure. */
int main (void)
{
    const bool commands = comms_test_commands();
    const bool telemetry = comms_test_telemetry();
    const bool timers = comms_test_timers();

    printf("S1-U04 commands: %s\n", commands ? "PASS" : "FAIL");
    printf("S1-U05 telemetry: %s\n", telemetry ? "PASS" : "FAIL");
    printf("S1-U06 timers: %s\n", timers ? "PASS" : "FAIL");

    return (commands && telemetry && timers) ? 0 : 1;
}

/*** End of file. ***/

