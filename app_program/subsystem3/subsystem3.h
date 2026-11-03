/**
 * @file subsystem3.h
 * @brief Subsystem 3: IR line sensing, barcodes and navigation commands.
 *        Owner: Edmund, student ID 2501578.
 *
 * Other subsystems include "../subsystem3/subsystem3.h". The line and
 * barcode interface (IF-06) is in line_sensor.h, with the command and mode
 * types in barcode.h and line_track.h.
 *
 * NOTE: The starter consumer example (IF-01, IF-04) stays until
 * subsystems 2 to 4 replace their examples together, because their code
 * still uses its objects. Its names (APP_STATS, mbx_samples, ...) come
 * from the starter template and do not follow BARR-C rules 5.1 and 7.1.
 */
#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"
#include "barcode.h"
#include "line_sensor.h"
#include "line_track.h"

/* Task priorities within subsystem 3's band, 16 to 17 (TEAM.md). */
#define S3_PRI_LINE         (16)
#define S3_PRI_CONSUMER     (17)

/* ---------------------------------------------------------------------
 *  Starter example (consumer), still used by subsystems 2 and 4.
 * --------------------------------------------------------------------- */
#define S3_BATCH_SIZE       (5u)    /* Report to subsystem 4 every N.    */

/* IF-01: send SAMPLE_MSG (subsystem2.h) here with tk_snd_mbx(). */
extern ID volatile mbx_samples;

/* IF-04: statistics. Read them only while holding mtx_stats:
 *  tk_loc_mtx(mtx_stats, TMO_FEVR); copy = app_stats; tk_unl_mtx(mtx_stats);
 */
typedef struct
{
    UW  received;   /* Samples received so far. */
    INT min;
    INT max;
    W   sum;
} APP_STATS;

extern ID volatile        mtx_stats;
extern APP_STATS volatile app_stats;

/** Starter example task that collects samples from subsystem 2. */
extern void consumer_task (INT stacd, void * p_exinf);

#endif /* SUBSYSTEM3_H */

/* End of file. */

