/**
 * @file s3_follow.h
 * @brief Subsystem 3 mission loop: front line reports and barcode commands.
 */
#ifndef S3_FOLLOW_H
#define S3_FOLLOW_H

/**
 * Run the line sensors and the barcode reader for the rest of the run.
 * Reports the line to the Vehicle Controller every 10 ms period (IF-06)
 * and each decoded barcode letter (IF-21). It never drives the motors.
 * Returns only if the start-up checks fail, after requesting a STOP.
 */
void s3_follow_run(void);

#endif /* S3_FOLLOW_H */

/* End of file. */
