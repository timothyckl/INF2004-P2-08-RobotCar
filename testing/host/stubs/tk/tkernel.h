/**
 * @file tkernel.h
 * @brief Host stand-in for <tk/tkernel.h>, for pure-logic tests only.
 *
 * Declares the micro T-Kernel types and calls that application logic
 * modules use, with host-sized fixed-width types. It is not an RTOS: the
 * calls in kernel_stub.c only record what was asked or fail with
 * E_NOSPT. Firmware never includes this file (testing/host only).
 */
#ifndef HOST_TKERNEL_H
#define HOST_TKERNEL_H

#include <stddef.h>   /* NULL, as the kernel headers provide */
#include <stdint.h>

typedef int8_t    B;
typedef int16_t   H;
typedef int32_t   W;
typedef uint8_t   UB;
typedef uint16_t  UH;
typedef uint32_t  UW;
typedef int       INT;
typedef unsigned  UINT;
typedef INT       BOOL;
typedef INT       ER;
typedef INT       ID;
typedef INT       PRI;
typedef W         TMO;
typedef UW        RELTIM;
typedef void      (*FP)(void);

#define IMPORT  extern
#define EXPORT
#define LOCAL   static

#ifndef TRUE
#define TRUE    1
#endif
#ifndef FALSE
#define FALSE   0
#endif

#define E_OK        (0)
#define E_SYS       (-5)
#define E_NOSPT     (-9)
#define E_PAR       (-17)
#define E_ID        (-18)
#define E_OBJ       (-41)
#define E_TMOUT     (-50)

#define TMO_POL     (0)
#define TMO_FEVR    (-1)

#define TSK_SELF    (0)
#define TP_PRC1     (0x1u)
#define TP_PRC2     (0x2u)

#define TA_TFIFO    (0x0u)
#define TA_INHERIT  (0x2u)
#define TA_WSGL     (0x0u)
#define TA_HLNG     (0x1u)
#define TWF_ORW     (0x1u)
#define TWF_CLR     (0x10u)
#define TWF_BITCLR  (0x20u)

typedef struct
{
    void * msgque[1];
} T_MSG;

typedef struct
{
    W  hi;
    UW lo;
} SYSTIM;

typedef struct
{
    void * exinf;
    UINT   wtsk;
    UINT   flgptn;
} T_RFLG;

ER tk_get_tim (SYSTIM * p_tim);
ER tk_set_flg (ID flgid, UINT setptn);
ER tk_clr_flg (ID flgid, UINT clrptn);
ER tk_ref_flg (ID flgid, T_RFLG * p_rflg);
ER tk_wai_flg (ID flgid, UINT waiptn, UINT wfmode, UINT * p_flgptn,
               TMO tmout);
ER tk_snd_mbf (ID mbfid, void const * p_msg, INT msgsz, TMO tmout);
INT tk_rcv_mbf (ID mbfid, void * p_msg, TMO tmout);
ER tk_loc_mtx (ID mtxid, TMO tmout);
ER tk_unl_mtx (ID mtxid);
ER tk_wup_tsk (ID tskid);
ER tk_sig_sem (ID semid, INT cnt);
ER tk_rel_mpf (ID mpfid, void * p_blf);

/* Host clock used by the tk_get_tim() stand-in. */
void host_set_time_ms (uint32_t now_ms);

#endif /* HOST_TKERNEL_H */

/* End of file. */
