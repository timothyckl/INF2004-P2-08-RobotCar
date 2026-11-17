/**
 * @file barcode_drive_test.h
 * @brief Controlled-drive barcode diagnostic interface.
 */

#ifndef BARCODE_DRIVE_TEST_H
#define BARCODE_DRIVE_TEST_H

#include "../app.h"

/** Start the GP20 control and barcode observation task. */
ER s3_barcode_drive_test_start(void);

#endif /* BARCODE_DRIVE_TEST_H */

