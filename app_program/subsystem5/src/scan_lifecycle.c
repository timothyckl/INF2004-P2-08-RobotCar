/**
 * @file scan_lifecycle.c
 * @brief Application lifecycle compatibility hooks for obstacle scanning.
 * Subsystem 5 -- Timothy.  Student ID: 2501530.
 *
 * Production scan logic is implemented in focused modules and exercised
 * through injected test ports. scan_task.c binds it to the servo, the
 * HC-SR04 and the Vehicle Controller (scan_q, controller_flags).
 */

#include "../include/scan_lifecycle.h"

#include "../include/scan_task.h"

#if APP_TEST
#include "../include/obstacle_test.h"
#endif

/**
 * @brief Create the scan task's objects without starting hardware or tasks.
 *
 * @return E_OK, or the kernel error.
 */
EXPORT ER
scan_lifecycle_init (void)
{
    return (scan_task_create());
}

/**
 * @brief Start the drivers and the scan task after initialization.
 *
 * @return E_OK, or the driver or kernel error.
 */
EXPORT ER
scan_lifecycle_start (void)
{
    return (scan_task_start());
}

#if APP_TEST
/** Run the isolated obstacle-scanning cases in test firmware. */
void
scan_lifecycle_test (void)
{
    obstacle_test_run();
}
#endif

/* End of file. */
