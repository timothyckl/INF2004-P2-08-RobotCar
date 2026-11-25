/**
 * @file kernel_model.c
 * @brief A small working model of the micro T-Kernel calls the Vehicle
 *        Controller uses, for the host integration suite only.
 *
 * Unlike kernel_stub.c (which fails every object call), this keeps real
 * state, single-threaded:
 *  - message buffers are FIFOs whose free space is counted as in
 *    kernel/tkernel/messagebuf.c: each message takes a 4-byte header plus
 *    its size rounded up to 4 bytes, so a full queue fails as on the car;
 *  - event flags keep a pattern; a wait never blocks: it returns the
 *    pattern at once if it matches, otherwise E_TMOUT (the test drives the
 *    10 ms periods itself);
 *  - mutexes always lock (one thread);
 *  - the clock is set by the test (ms for tk_get_tim, us for TIMERAWL);
 *  - tm_printf keeps the console text so a test can search it, and
 *    prints it too when HOST_VERBOSE is set in the environment.
 * Firmware never uses this file.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "kernel_model.h"

#define MODEL_MBF_MAX       8
#define MODEL_MBF_BYTES     256
#define MODEL_FLG_MAX       4
#define MODEL_HEADER_BYTES  4
#define MODEL_CONSOLE_BYTES 65536

typedef struct
{
    UB   data[MODEL_MBF_BYTES];
    INT  head;          /* read offset */
    INT  used;          /* bytes in use, kernel accounting */
    INT  bufsz;
    INT  maxmsz;
    bool b_used;
} model_mbf_t;

typedef struct
{
    UINT pattern;
    bool b_used;
} model_flg_t;

static model_mbf_t g_mbf[MODEL_MBF_MAX];
static model_flg_t g_flg[MODEL_FLG_MAX];
static ID          g_mtx_count = 0;
static uint32_t    g_now_ms = 0u;
static uint32_t    g_now_us = 0u;
static UW          g_gpio_low = 0u;    /* inputs pulled low (pressed) */
static char        g_console[MODEL_CONSOLE_BYTES];
static size_t      g_console_len = 0u;

static INT model_round (INT size);

void
host_set_time_ms (uint32_t now_ms)
{
    g_now_ms = now_ms;
    g_now_us = now_ms * 1000u;
}

void
model_reset (void)
{
    memset(g_mbf, 0, sizeof(g_mbf));
    memset(g_flg, 0, sizeof(g_flg));
    g_mtx_count = 0;
    g_now_ms = 0u;
    g_now_us = 0u;
    g_gpio_low = 0u;
    model_console_clear();
}

void
model_console_clear (void)
{
    g_console_len = 0u;
    g_console[0] = '\0';
}

bool
model_console_has (char const * p_text)
{
    return (NULL != strstr(g_console, p_text));
}

INT
model_mbf_count (ID mbfid)
{
    INT count = 0;

    if ((mbfid > 0) && (mbfid <= MODEL_MBF_MAX) && g_mbf[mbfid - 1].b_used)
    {
        model_mbf_t const * const p_mbf = &g_mbf[mbfid - 1];
        INT offset = p_mbf->head;
        INT left = p_mbf->used;

        while (left > 0)
        {
            INT size;

            memcpy(&size, &p_mbf->data[offset % MODEL_MBF_BYTES],
                   sizeof(size));
            offset += MODEL_HEADER_BYTES + model_round(size);
            left -= MODEL_HEADER_BYTES + model_round(size);
            count++;
        }
    }

    return (count);
}

UINT
model_flag_pattern (ID flgid)
{
    return (((flgid > 0) && (flgid <= MODEL_FLG_MAX))
            ? g_flg[flgid - 1].pattern : 0u);
}

ID
tk_cre_flg (T_CFLG const * p_cflg)
{
    for (ID idx = 0; idx < MODEL_FLG_MAX; idx++)
    {
        if (!g_flg[idx].b_used)
        {
            g_flg[idx].b_used = true;
            g_flg[idx].pattern = p_cflg->iflgptn;
            return (idx + 1);
        }
    }

    return (E_NOEXS);
}

ID
tk_cre_mbf (T_CMBF const * p_cmbf)
{
    for (ID idx = 0; idx < MODEL_MBF_MAX; idx++)
    {
        if (!g_mbf[idx].b_used)
        {
            if (model_round(p_cmbf->bufsz) > MODEL_MBF_BYTES)
            {
                return (E_PAR);
            }
            g_mbf[idx].b_used = true;
            g_mbf[idx].bufsz = model_round(p_cmbf->bufsz);
            g_mbf[idx].maxmsz = p_cmbf->maxmsz;
            g_mbf[idx].head = 0;
            g_mbf[idx].used = 0;
            return (idx + 1);
        }
    }

    return (E_NOEXS);
}

ID
tk_cre_mtx (T_CMTX const * p_cmtx)
{
    (void)p_cmtx;
    g_mtx_count++;
    return (g_mtx_count);
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
    if ((flgid <= 0) || (flgid > MODEL_FLG_MAX) || !g_flg[flgid - 1].b_used)
    {
        return (E_ID);
    }
    g_flg[flgid - 1].pattern |= setptn;
    return (E_OK);
}

ER
tk_clr_flg (ID flgid, UINT clrptn)
{
    if ((flgid <= 0) || (flgid > MODEL_FLG_MAX) || !g_flg[flgid - 1].b_used)
    {
        return (E_ID);
    }
    g_flg[flgid - 1].pattern &= clrptn;     /* the kernel ANDs the mask */
    return (E_OK);
}

ER
tk_ref_flg (ID flgid, T_RFLG * p_rflg)
{
    if ((flgid <= 0) || (flgid > MODEL_FLG_MAX) || !g_flg[flgid - 1].b_used)
    {
        return (E_ID);
    }
    p_rflg->exinf = NULL;
    p_rflg->wtsk = 0u;
    p_rflg->flgptn = g_flg[flgid - 1].pattern;
    return (E_OK);
}

ER
tk_wai_flg (ID flgid, UINT waiptn, UINT wfmode, UINT * p_flgptn, TMO tmout)
{
    (void)tmout;

    if ((flgid <= 0) || (flgid > MODEL_FLG_MAX) || !g_flg[flgid - 1].b_used)
    {
        return (E_ID);
    }
    if (0u == (g_flg[flgid - 1].pattern & waiptn))
    {
        return (E_TMOUT);   /* never blocks: the test advances time */
    }
    *p_flgptn = g_flg[flgid - 1].pattern;
    if (0u != (wfmode & TWF_CLR))
    {
        g_flg[flgid - 1].pattern = 0u;
    }
    return (E_OK);
}

ER
tk_snd_mbf (ID mbfid, void const * p_msg, INT msgsz, TMO tmout)
{
    model_mbf_t * p_mbf;
    INT need;
    INT tail;

    (void)tmout;    /* never blocks: a full queue fails as with TMO_POL */
    if ((mbfid <= 0) || (mbfid > MODEL_MBF_MAX) || !g_mbf[mbfid - 1].b_used)
    {
        return (E_ID);
    }
    p_mbf = &g_mbf[mbfid - 1];
    if ((msgsz <= 0) || (msgsz > p_mbf->maxmsz))
    {
        return (E_PAR);
    }
    need = MODEL_HEADER_BYTES + model_round(msgsz);
    if ((p_mbf->used + need) > p_mbf->bufsz)
    {
        return (E_TMOUT);
    }
    tail = p_mbf->head + p_mbf->used;
    for (INT idx = 0; idx < MODEL_HEADER_BYTES; idx++)
    {
        p_mbf->data[(tail + idx) % MODEL_MBF_BYTES] = ((UB const *)&msgsz)[idx];
    }
    for (INT idx = 0; idx < msgsz; idx++)
    {
        p_mbf->data[(tail + MODEL_HEADER_BYTES + idx) % MODEL_MBF_BYTES] =
            ((UB const *)p_msg)[idx];
    }
    p_mbf->used += need;
    return (E_OK);
}

INT
tk_rcv_mbf (ID mbfid, void * p_msg, TMO tmout)
{
    model_mbf_t * p_mbf;
    INT size = 0;

    (void)tmout;
    if ((mbfid <= 0) || (mbfid > MODEL_MBF_MAX) || !g_mbf[mbfid - 1].b_used)
    {
        return (E_ID);
    }
    p_mbf = &g_mbf[mbfid - 1];
    if (0 == p_mbf->used)
    {
        return (E_TMOUT);
    }
    for (INT idx = 0; idx < MODEL_HEADER_BYTES; idx++)
    {
        ((UB *)&size)[idx] = p_mbf->data[(p_mbf->head + idx) % MODEL_MBF_BYTES];
    }
    for (INT idx = 0; idx < size; idx++)
    {
        ((UB *)p_msg)[idx] =
            p_mbf->data[(p_mbf->head + MODEL_HEADER_BYTES + idx)
                        % MODEL_MBF_BYTES];
    }
    p_mbf->head = (p_mbf->head + MODEL_HEADER_BYTES + model_round(size))
                  % MODEL_MBF_BYTES;
    p_mbf->used -= MODEL_HEADER_BYTES + model_round(size);
    return (size);
}

ER
tk_loc_mtx (ID mtxid, TMO tmout)
{
    (void)mtxid;
    (void)tmout;
    return (E_OK);
}

ER
tk_unl_mtx (ID mtxid)
{
    (void)mtxid;
    return (E_OK);
}

ER
tk_wup_tsk (ID tskid)
{
    (void)tskid;
    return (E_OK);
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
    /* GPIO_IN: every input high (released, pulled up) except a pressed
       button, which reads low. Anything else is the 1 MHz timer. */
    return ((GPIO_IN == addr) ? ~g_gpio_low : g_now_us);
}

void
out_w (UW addr, UW value)
{
    (void)addr;
    (void)value;
}

void
set_w (UW addr, UW value)
{
    (void)addr;
    (void)value;
}

void
clr_w (UW addr, UW value)
{
    (void)addr;
    (void)value;
}

ER
gpio_set_pin (UINT no, UINT mode)
{
    (void)no;
    (void)mode;
    return (E_OK);
}

void
model_gpio_low (UW mask)
{
    g_gpio_low = mask;
}

INT
tm_printf (UB const * p_format, ...)
{
    va_list args;
    INT     n;
    char    line[512];

    va_start(args, p_format);
    n = vsnprintf(line, sizeof(line), (char const *)p_format, args);
    va_end(args);

    if ((g_console_len + strlen(line) + 1u) < sizeof(g_console))
    {
        memcpy(&g_console[g_console_len], line, strlen(line) + 1u);
        g_console_len += strlen(line);
    }
    if (NULL != getenv("HOST_VERBOSE"))
    {
        fputs(line, stdout);
    }

    return (n);
}

static INT
model_round (INT size)
{
    return ((size + (MODEL_HEADER_BYTES - 1)) & ~(MODEL_HEADER_BYTES - 1));
}

/* End of file. */
