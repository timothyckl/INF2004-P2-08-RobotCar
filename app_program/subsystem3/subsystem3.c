/*
 *	subsystem3.c
 *	Subsystem 3 -- creates its objects and starts its task: the Line
 *	Sensor task (task_line.c, IF-06). The starter consumer (IF-01, IF-04)
 *	was retired when the Line Sensor task replaced it.
 */

#include "subsystem3.h"

EXPORT ER subsystem3_init(void)
{
	return s3_line_create();
}

EXPORT ER subsystem3_start(void)
{
	return s3_line_start();
}
