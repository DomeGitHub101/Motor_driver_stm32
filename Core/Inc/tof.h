/**
  ******************************************************************************
  * @file    tof.h
  * @brief   Front distance sensor interface (sensor model not chosen yet).
  ******************************************************************************
  */
#ifndef TOF_H
#define TOF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define TOF_NO_OBJECT_MM  2000U   /* returned when nothing is in range */

void     tof_init(void);
uint16_t tof_read_mm(void);

/* Test hook: force tof_read_mm() to return mm (< 0 = use the real sensor). */
void     tof_set_override(int mm);
int      tof_get_override(void);

#ifdef __cplusplus
}
#endif

#endif /* TOF_H */
