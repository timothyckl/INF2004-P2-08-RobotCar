/**
 * @file s5_host_main.c
 * @brief Runs subsystem 5's unit cases (obstacle_test_run) on the host.
 *        The cases and fixture are the ones the Pico W TEST=1 image runs.
 */
#include "../../app_program/subsystem5/include/obstacle_test.h"

int
main (void)
{
    obstacle_test_run();

    return (0);
}

/* End of file. */
