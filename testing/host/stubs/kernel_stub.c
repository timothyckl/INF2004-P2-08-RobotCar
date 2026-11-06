/**
 * @file kernel_stub.c
 * @brief Host stand-ins for the kernel calls pure-logic modules link to.
 *
 * Object calls fail with E_NOSPT: a test that reaches one is testing
 * kernel glue, which belongs on the Pico W (TEST=1), not here.
 */
#include <stdarg.h>
#include <stdio.h>

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

static uint32_t g_now_ms = 0u;

void
host_set_time_ms (uint32_t now_ms)
{
    g_now_ms = now_ms;
}

ER
tk_get_tim (SYSTIM * p_tim)
{
    p_tim->hi = 0;
    p_tim->lo = g_now_ms;
    return (E_OK);
}

ER
tk_set_flg (ID flgid, UINT setptn)
{
    (void)flgid;
    (void)setptn;
    return (E_NOSPT);
}

ER
tk_clr_flg (ID flgid, UINT clrptn)
{
    (void)flgid;
    (void)clrptn;
    return (E_NOSPT);
}

ER
tk_ref_flg (ID flgid, T_RFLG * p_rflg)
{
    (void)flgid;
    (void)p_rflg;
    return (E_NOSPT);
}

ER
tk_wai_flg (ID flgid, UINT waiptn, UINT wfmode, UINT * p_flgptn, TMO tmout)
{
    (void)flgid;
    (void)waiptn;
    (void)wfmode;
    (void)tmout;
    *p_flgptn = 0u;
    return (E_NOSPT);
}

ER
tk_snd_mbf (ID mbfid, void const * p_msg, INT msgsz, TMO tmout)
{
    (void)mbfid;
    (void)p_msg;
    (void)msgsz;
    (void)tmout;
    return (E_NOSPT);
}

INT
tk_rcv_mbf (ID mbfid, void * p_msg, TMO tmout)
{
    (void)mbfid;
    (void)p_msg;
    (void)tmout;
    return (E_NOSPT);
}

ER
tk_loc_mtx (ID mtxid, TMO tmout)
{
    (void)mtxid;
    (void)tmout;
    return (E_NOSPT);
}

ER
tk_unl_mtx (ID mtxid)
{
    (void)mtxid;
    return (E_NOSPT);
}

ER
tk_wup_tsk (ID tskid)
{
    (void)tskid;
    return (E_NOSPT);
}

ER
tk_sig_sem (ID semid, INT cnt)
{
    (void)semid;
    (void)cnt;
    return (E_NOSPT);
}

ER
tk_rel_mpf (ID mpfid, void * p_blf)
{
    (void)mpfid;
    (void)p_blf;
    return (E_NOSPT);
}

UW
in_w (UW addr)
{
    (void)addr;
    return (0u);
}

void
out_w (UW addr, UW value)
{
    (void)addr;
    (void)value;
}

INT
tm_printf (UB const * p_format, ...)
{
    va_list args;
    INT     n;

    va_start(args, p_format);
    n = vprintf((char const *)p_format, args);
    va_end(args);

    return (n);
}

/* End of file. */
