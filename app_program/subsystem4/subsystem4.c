/**
 * @file subsystem4.c
 * @brief Kernel object lifecycle and status access for Subsystem 4.
 */
#include "subsystem4_internal.h"

static ID s4_status_mutex;
static ID s4_sample_cyclic_id;
static ID s4_task_id;
static s4_terrain_result_t s4_status;

static void s4_sample_cyclic(void * p_exinf);

EXPORT ER
subsystem4_init (void)
{
    T_CMTX mutex_config = {0};
    T_CCYC cyclic_config = {0};
    ER error = E_OK;

    s4_status = (s4_terrain_result_t) {0};
    s4_status.state = S4_TERRAIN_STATE_CALIBRATING;

    mutex_config.mtxatr = TA_INHERIT;
    s4_status_mutex = tk_cre_mtx(&mutex_config);
    if (s4_status_mutex < E_OK)
    {
        error = s4_status_mutex;
    }

    /* A cyclic handler sets the sample period: tk_dly_tsk() adds one
       timer tick, so a 10 ms delay sleeps about 20 ms. */
    if (E_OK == error)
    {
        cyclic_config.cycatr = TA_HLNG;     /* started with the task */
        cyclic_config.cychdr = (FP) s4_sample_cyclic;
        cyclic_config.cyctim = S4_SAMPLE_PERIOD_MS;
        cyclic_config.cycphs = S4_SAMPLE_PERIOD_MS;
        s4_sample_cyclic_id = tk_cre_cyc(&cyclic_config);
        if (s4_sample_cyclic_id < E_OK)
        {
            error = s4_sample_cyclic_id;
        }
    }

    return (error);
}

EXPORT ER
subsystem4_start (void)
{
    ER error = E_OK;
    ID task_id = app_start_task((FP) s4_imu_task, S4_PRI_IMU, 0u,
                                "s4_imu_task");

    if (task_id < E_OK)
    {
        error = task_id;
    }
    else
    {
        s4_task_id = task_id;
        error = tk_sta_cyc(s4_sample_cyclic_id);
    }

    return (error);
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

/* Timer context: wakes the IMU task once per sample period. */
static void
s4_sample_cyclic (void * p_exinf)
{
    (void) p_exinf;

    if (s4_task_id > 0)
    {
        (void) tk_wup_tsk(s4_task_id);
    }
}

/* End of file. */
