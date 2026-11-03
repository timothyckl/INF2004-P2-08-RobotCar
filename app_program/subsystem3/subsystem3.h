/**
 * @file subsystem3.h
 * @brief Public interface for subsystem 3 and retained starter objects.
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"
#include "line_sensor.h"

#define S3_PRI_CONSUMER (17)
#define S3_BATCH_SIZE   (5U)

/* Retained IF-01 starter mailbox from subsystem 2. */
IMPORT ID mbx_samples;

/** Retained statistics used by the starter monitor task. */
typedef struct
{
    UW  received;
    INT min;
    INT max;
    W   sum;
} APP_STATS;

IMPORT ID        mtx_stats;
IMPORT APP_STATS app_stats;

/* Retained starter task entry point. */
IMPORT void consumer_task(INT start_code, void * p_extension);

#endif /* SUBSYSTEM3_H */

/* End of file. */
