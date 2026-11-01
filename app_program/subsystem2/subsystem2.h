/*
 *	subsystem2.h
 *	Subsystem 2 -- Producer (sample source).  Student ID: <student ID>.
 *
 *	Public interface of subsystem 2.  Other subsystems include it as
 *	"../subsystem2/subsystem2.h".
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"

/* ---------------------------------------------------------------------
 *  PROVISIONAL robot interface (completed in Step 3, agreed in TEAM.md).
 *  controller_flags: one bit per urgent event; setters use tk_set_flg,
 *  only the Vehicle Controller clears, one handled bit at a time.
 *  safety_flags: the latched STOP, set and cleared only by the Vehicle
 *  Controller; anyone may read it with tk_ref_flg.
 * --------------------------------------------------------------------- */
#define S2_PRI_CONTROLLER	14	/* highest application priority */
#define S2_PRI_MOTION		15

#define S2_CFLG_HAZARD		(1u << 0)	/* set by S5 */
#define S2_CFLG_SCAN_FAULT	(1u << 1)	/* set by S5 */
#define S2_CFLG_MOTION_FAULT	(1u << 2)	/* set by S2 Motion Control */
#define S2_CFLG_REMOTE_STOP	(1u << 3)	/* set by S1 */

#define S2_SFLG_STOP		(1u << 0)

IMPORT ID	controller_flags;
IMPORT ID	safety_flags;

/* ---------------------------------------------------------------------
 *  LEGACY STARTER (IF-01, IF-02): kept while subsystems 3 and 4 use it.
 * --------------------------------------------------------------------- */
#define S2_PRI_PRODUCER		14	/* priority band 14-15 (TEAM.md) */
#define S2_PERIOD_MS		300	/* one sample every 300 ms */

/* ---------------------------------------------------------------------
 *  IF-01 data format: one sample, sent to subsystem 3's mailbox.
 *  A mailbox message MUST start with a T_MSG header: the kernel links
 *  messages together through it, so the data itself is never copied.
 * --------------------------------------------------------------------- */
typedef struct {
	T_MSG	hdr;		/* used by the kernel, do not touch */
	UW	seq;		/* sample sequence number */
	UW	time_ms;	/* system time when the sample was taken */
	INT	value;		/* the "measurement", 0..100 */
	INT	prc;		/* processor (core) that produced it: 1 or 2 */
} SAMPLE_MSG;

#define S2_POOL_COUNT	4	/* at most 4 samples in flight */

/* IF-02: the receiver of a sample gives its memory block back to
   mpf_samples (tk_rel_mpf) and then a slot back to sem_slots
   (tk_sig_sem, count 1).  Nothing else may use these two objects. */
IMPORT ID	sem_slots;
IMPORT ID	mpf_samples;

/* Private to subsystem 2 */
IMPORT void	producer_task(INT stacd, void *exinf);
IMPORT INT	s2_make_value(UW seq);

#endif /* SUBSYSTEM2_H */
