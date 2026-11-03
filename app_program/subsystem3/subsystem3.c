/**
 * @file subsystem3.c
 * @brief Subsystem 3: creates its kernel objects and starts its tasks.
 */
#include "subsystem3.h"

ID volatile        mbx_samples = 0;     /* IF-01, starter example. */
ID volatile        mtx_stats   = 0;     /* IF-04, starter example. */
APP_STATS volatile app_stats;

/** Create the starter example's mailbox and mutex. */
static ER s3_create_example_objects (void);

ER
subsystem3_init (void)
{
    ER err = line_sensor_init();

    if (E_OK == err)
    {
        err = s3_create_example_objects();
    }

    return (err);
}

ER
subsystem3_start (void)
{
    ER err = line_sensor_start(S3_PRI_LINE);

    if (E_OK == err)
    {
        ID const h_consumer = app_start_task(consumer_task, S3_PRI_CONSUMER,
                                             TP_PRC1, "consumer_task");

        err = (h_consumer < E_OK) ? h_consumer : E_OK;
    }

    return (err);
}

static ER
s3_create_example_objects (void)
{
    T_CMBX cmbx = {0};
    T_CMTX cmtx = {0};
    ER     err  = E_OK;

    cmbx.mbxatr = (TA_TFIFO | TA_MFIFO);
    mbx_samples = tk_cre_mbx(&cmbx);

    if (mbx_samples < E_OK)
    {
        err = mbx_samples;
    }
    else
    {
        cmtx.mtxatr = TA_INHERIT;
        mtx_stats   = tk_cre_mtx(&cmtx);

        err = (mtx_stats < E_OK) ? mtx_stats : E_OK;
    }

    return (err);
}

/* End of file. */

