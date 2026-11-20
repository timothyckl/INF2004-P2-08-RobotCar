/**
 * @file subsystem4.c
 * @brief Kernel object lifecycle and status access for Subsystem 4.
 */
#include "subsystem4_internal.h"

static ID s4_status_mutex;
static s4_terrain_result_t s4_status;

EXPORT ER
subsystem4_init (void)
{
    T_CMTX mutex_config = {0};
    ER error = E_OK;

    s4_status = (s4_terrain_result_t) {0};
    s4_status.state = S4_TERRAIN_STATE_CALIBRATING;

    mutex_config.mtxatr = TA_INHERIT;
    s4_status_mutex = tk_cre_mtx(&mutex_config);
    if (s4_status_mutex < E_OK)
    {
        error = s4_status_mutex;
    }

    return (error);
}

EXPORT ER
subsystem4_start (void)
{
    ID task_id = app_start_task((FP) s4_imu_task, S4_PRI_IMU, 0u,
                                "s4_imu_task");

#if S4_RAW_LOG
    if (task_id >= E_OK)
    {
        ID const watch_id = app_start_task((FP) s4_watch_task,
                                           S4_PRI_IMU + 1, 0u,
                                           "s4_watch_task");

        s4_watch_add_imu(task_id);
        if (watch_id < E_OK)
        {
            task_id = watch_id;
        }
    }
#endif

    return ((task_id < E_OK) ? task_id : E_OK);
}

/*
 * Copy of the latest terrain result (IF-18). Never waits: subsystem 2's
 * telemetry reader runs in the WiFi service task, so a busy snapshot
 * returns E_TMOUT and the caller tries again later (S4 decision 13).
 */
ER
subsystem4_get_status (s4_terrain_result_t * p_result)
{
    ER error = E_PAR;

    if (NULL != p_result)
    {
        error = tk_loc_mtx(s4_status_mutex, TMO_POL);
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
