/**
 * @file subsystem3.c
 * @brief Create and start the subsystem 3 objects and legacy consumer task.
 */

#include "subsystem3.h"
#include "s3_ir_gpio.h"

EXPORT ID mbx_samples; /* IF-01 */
EXPORT ID mtx_stats;   /* IF-04 */
EXPORT APP_STATS app_stats;

EXPORT ER
subsystem3_init (void)
{
    T_CMBX mailbox_config = {0};
    T_CMTX mutex_config = {0};

    /* Inputs only: this test stage never commands either motor. */
    s3_ir_gpio_init();

    /*
     * The mailbox passes a pointer without copying its message. Subsystem 2
     * therefore allocates message storage from its memory pool.
     */
    mailbox_config.mbxatr = TA_TFIFO | TA_MFIFO;
    mbx_samples = tk_cre_mbx(&mailbox_config);
    if (mbx_samples < E_OK)
    {
        return (mbx_samples);
    }

    /* Priority inheritance protects the shared legacy statistics. */
    mutex_config.mtxatr = TA_INHERIT;
    mtx_stats = tk_cre_mtx(&mutex_config);

    return ((mtx_stats < E_OK) ? mtx_stats : E_OK);
}

EXPORT ER
subsystem3_start (void)
{
    ID const task_id = app_start_task(consumer_task, S3_PRI_CONSUMER,
                                      TP_PRC1, "consumer_task");

    return ((task_id < E_OK) ? task_id : E_OK);
}

/* End of file. */
