/*
 *	subsystem3.h
 *	Subsystem 3 -- IR line sensing, barcodes and navigation commands.
 *	Owner: Edmund, student ID 2501578.
 *
 *	Public interface of subsystem 3.  Other subsystems include it as
 *	"../subsystem3/subsystem3.h".
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include <stdint.h>

#include "../app.h"

/* ---------------------------------------------------------------------
 *  IF-06: Line Sensor -> Vehicle Controller (subsystem 2).
 *
 *  The line sensor task samples the two front sensors and the side
 *  barcode sensor every 10 ms and publishes one status snapshot under its
 *  own mutex. The controller copies it with s3_line_get(), which never
 *  waits. Events are wrapping counters, not one-shot bits, so a reader
 *  that skips a sample cannot lose or repeat one: compare with the last
 *  copy by unsigned subtraction.
 *
 *  Navigation: S3 stages each decoded barcode command and offers it only
 *  at a junction while robot_state has S2_RS_NAV_AUTHORISED and STOP is
 *  clear (s3_nav_take, as subsystem2.h describes). An offered command is
 *  published as nav_cmd with a new nav_id; the controller executes each
 *  nav_id at most once. STOP, expiry or a missing authorisation discard
 *  the staged command instead.
 * --------------------------------------------------------------------- */
#define S3_PRI_LINE		16	/* priority band 16-17 (TEAM.md) */

#define S3_ST_VALID		0x01u	/* line decision settled, usable */
#define S3_ST_LOST		0x02u	/* no front sensor black for lost_ms */
#define S3_ST_JUNCTION		0x04u	/* settled decision is a junction */
#define S3_ST_FAULT		0x08u	/* both black too long; latched */
#define S3_ST_SEEN		0x10u	/* a front sensor sees black now */
#define S3_ST_BAR_BLACK		0x20u	/* side barcode sensor sees black */

typedef struct
{
	uint32_t captured_ms;	/* tk_get_tim() low 32 bits of the sample */
	uint32_t seq;		/* samples since start, wraps */
	uint32_t nav_id;	/* ID of nav_cmd; 0 until the first offer */
	int16_t  steer_permille; /* +left/-right, |x| <= 1000; 0 unless VALID */
	uint8_t  line;		/* s3_line_t (s3_ir.h) */
	uint8_t  flags;		/* S3_ST_* */
	uint8_t  last_side;	/* S3_TRACK_SIDE_* (s3_track.h) */
	uint8_t  nav_cmd;	/* S3_NAV_* = S2_NAV_* values, with nav_id */
	uint8_t  last_symbol;	/* last decoded barcode symbol, 0 if none */
	uint8_t  junctions;	/* junction events, wraps */
	uint8_t  barcodes;	/* decoded barcodes, wraps */
	uint8_t  barcode_errors; /* failed captures, wraps */
	uint8_t  losses;	/* line-lost events, wraps */
	uint8_t  overruns;	/* missed 10 ms sample periods, wraps */
} s3_line_status_t;

/* Copy the latest snapshot. E_OK, E_PAR, E_OBJ before subsystem3_init(),
   or E_TMOUT if the writer holds the lock (keep the previous copy). */
IMPORT ER	s3_line_get(s3_line_status_t *p_status);

/* ---------------------------------------------------------------------
 *  LEGACY STARTER (IF-01, IF-04): kept while subsystems 2 and 4 use it.
 * --------------------------------------------------------------------- */
#define S3_PRI_CONSUMER		17	/* priority band 16-17 (TEAM.md) */
#define S3_BATCH_SIZE		5	/* report to subsystem 4 every N samples */

/* IF-01: send SAMPLE_MSG (subsystem2.h) here with tk_snd_mbx */
IMPORT ID	mbx_samples;

/* IF-04: statistics.  Read them ONLY while holding mtx_stats:
 *	tk_loc_mtx(mtx_stats, TMO_FEVR);  copy = app_stats;  tk_unl_mtx(mtx_stats);
 */
typedef struct {
	UW	received;	/* samples received so far */
	INT	min;
	INT	max;
	W	sum;
} APP_STATS;

IMPORT ID		mtx_stats;
IMPORT APP_STATS	app_stats;

/* Private to subsystem 3 */
IMPORT void	consumer_task(INT stacd, void *exinf);
IMPORT ER	s3_line_create(void);	/* task_line.c, from subsystem3_init */
IMPORT ER	s3_line_start(void);	/* task_line.c, from subsystem3_start */

#endif /* SUBSYSTEM3_H */
