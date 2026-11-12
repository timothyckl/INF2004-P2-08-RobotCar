/**
 * @file subsystem4.c
 * @brief Kernel object lifecycle and status access for Subsystem 4.
 */
#include "subsystem4_internal.h"

EXPORT ID mbf_summary;
EXPORT ID flg_events;

static ID s4_status_mutex;
static s4_terrain_result_t s4_status;

EXPORT ER
subsystem4_init (void)
{
    T_CFLG flag_config = {0};
    T_CMBF buffer_config = {0};
    T_CMTX mutex_config = {0};
    ER error = E_OK;

    s4_status = (s4_terrain_result_t) {0};
    s4_status.state = S4_TERRAIN_STATE_CALIBRATING;

    /* Preserve the starter receiver objects while S3 still publishes them. */
    flag_config.flgatr = TA_TFIFO | TA_WMUL;
    flag_config.iflgptn = 0u;
    flg_events = tk_cre_flg(&flag_config);
    if (flg_events < E_OK)
    {
        error = flg_events;
    }

    if (E_OK == error)
    {
        buffer_config.mbfatr = TA_TFIFO;
        buffer_config.bufsz = 4 * S4_SUMMARY_MAX;
        buffer_config.maxmsz = S4_SUMMARY_MAX;
        mbf_summary = tk_cre_mbf(&buffer_config);
        if (mbf_summary < E_OK)
        {
            error = mbf_summary;
        }
    }

    if (E_OK == error)
    {
        mutex_config.mtxatr = TA_INHERIT;
        s4_status_mutex = tk_cre_mtx(&mutex_config);
        if (s4_status_mutex < E_OK)
        {
            error = s4_status_mutex;
        }
    }

    return (error);
}

EXPORT ER
subsystem4_start (void)
{
    ID task_id = app_start_task((FP) s4_imu_task, S4_PRI_IMU, 0u,
                                "s4_imu_task");

    return ((task_id < E_OK) ? task_id : E_OK);
}

ER
subsystem4_get_status (s4_terrain_result_t * p_result)
{
    ER error = E_PAR;

    if (NULL != p_result)
    {
        error = tk_loc_mtx(s4_status_mutex, TMO_FEVR);
        if (E_OK == error)
        {
            *p_result = s4_status;
            error = tk_unl_mtx(s4_status_mutex);
        }
    }

    return (error);
}

ER
s4_status_publish (s4_terrain_result_t const * p_result)
{
    ER error = E_PAR;

    if (NULL != p_result)
    {
        error = tk_loc_mtx(s4_status_mutex, TMO_FEVR);
        if (E_OK == error)
        {
            s4_status = *p_result;
            error = tk_unl_mtx(s4_status_mutex);
        }
    }

    return (error);
}

/* End of file. */
