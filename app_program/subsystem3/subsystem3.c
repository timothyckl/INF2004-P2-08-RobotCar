/**
 * @file subsystem3.c
 * @brief Subsystem 3 object creation and task startup.
 *
 * The public starter object names are retained for compatibility with the
 * remaining producer, consumer, and monitor examples.
 */

#include "subsystem3.h"

EXPORT ID        mbx_samples = 0;
EXPORT ID        mtx_stats = 0;
EXPORT APP_STATS app_stats = {0};

EXPORT ER
subsystem3_init (void)
{
    T_CMBX mailbox_config = {0};
    T_CMTX mutex_config = {0};
    ER error = E_OK;

    mailbox_config.mbxatr = TA_TFIFO | TA_MFIFO;
    mbx_samples = tk_cre_mbx(&mailbox_config);
    if (E_OK > mbx_samples)
    {
        error = mbx_samples;
    }

    if (E_OK == error)
    {
        mutex_config.mtxatr = TA_INHERIT;
        mtx_stats = tk_cre_mtx(&mutex_config);
        if (E_OK > mtx_stats)
        {
            error = mtx_stats;
        }
    }

    if (E_OK == error)
    {
        error = line_sensor_init();
    }

    return (error);
}

EXPORT ER
subsystem3_start (void)
{
    ID const task_id = app_start_task(consumer_task,
                                      S3_PRI_CONSUMER,
                                      TP_PRC1,
                                      "consumer_task");
    ER error = E_OK;

    if (E_OK > task_id)
    {
        error = task_id;
    }
    else
    {
        error = line_sensor_start();
    }

    return (error);
}

/* End of file. */
