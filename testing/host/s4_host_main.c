/**
 * @file s4_host_main.c
 * @brief Runs subsystem 4's logic and decode cases (subsystem4_test) on
 *        the host. The cases are the ones the Pico W TEST=1 image runs;
 *        S4-H01 needs the sensor and stays on the Pico W.
 */
#include "../../app_program/app.h"

int
main (void)
{
    subsystem4_test();

    return (0);
}

/* End of file. */
