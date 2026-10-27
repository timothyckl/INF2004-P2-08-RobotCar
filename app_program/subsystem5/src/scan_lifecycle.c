/**
 * @file scan_lifecycle.c
 * @brief Application lifecycle compatibility hooks for obstacle scanning.
 *
 * Production scan logic is implemented in focused modules and exercised
 * through injected test ports. No task or hardware is started here until
 * the controller interfaces and servo calibration have been agreed.
 */

#include "../include/scan_lifecycle.h"

#if APP_TEST
#include "../include/obstacle_test.h"
#endif

/**
 * @brief Initialize agreed objects without starting hardware or tasks.
 *
 * @return E_OK because no kernel objects are currently required.
 */
EXPORT ER
scan_lifecycle_init (void)
{
    return E_OK;
}

/**
 * @brief Start agreed subsystem tasks after initialization is complete.
 *
 * @return E_OK because no production task is currently enabled.
 */
EXPORT ER
scan_lifecycle_start (void)
{
    return E_OK;
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
