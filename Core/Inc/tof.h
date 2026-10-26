/**
  ******************************************************************************
  * @file    tof.h
  * @brief   Front distance sensor (one VL53L0X on I2C1, XSHUT = PA0).
  ******************************************************************************
  */
#ifndef TOF_H
#define TOF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

#define TOF_NO_OBJECT_MM  2000U   /* readings beyond this are clamped to it */
#define TOF_POLL_MS          5U   /* I2C status check period */
#define TOF_TIMEOUT_MS     300U   /* no new sample this long = sensor error */
#define TOF_RETRY_MS      2000U   /* re-init period while in error */

void     tof_init(void);          /* blocking, call once at boot */
void     tof_poll(void);          /* non-blocking, call from the main loop */
uint16_t tof_read_mm(void);       /* latest distance in mm */
bool     tof_ok(void);            /* false = sensor error (treat as level 3) */

/* Test hook: force tof_read_mm() to return mm (< 0 = use the real sensor). */
void     tof_set_override(int mm);
int      tof_get_override(void);

#ifdef __cplusplus
}
#endif

#endif /* TOF_H */
