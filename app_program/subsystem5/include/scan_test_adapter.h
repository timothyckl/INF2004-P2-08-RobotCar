/**
 * @file scan_test_adapter.h
 * @brief On-target test binding for obstacle scan logic.
 */
#ifndef SCAN_TEST_ADAPTER_H
#define SCAN_TEST_ADAPTER_H

#if APP_TEST
#include "test_interface.h"

/**
 * @brief Return the production operations used by the on-target tests.
 *
 * @return Complete test operations bound to production scan logic.
 */
test_interface_api_t const * scan_test_adapter_get (void);
#endif

#endif /* SCAN_TEST_ADAPTER_H */

/* End of file. */
