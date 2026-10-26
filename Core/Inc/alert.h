/**
  ******************************************************************************
  * @file    alert.h
  * @brief   Obstacle alert (buzzer / speaker). Hardware not chosen yet.
  ******************************************************************************
  */
#ifndef ALERT_H
#define ALERT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define ALERT_REPEAT_MS  3000U   /* level 3 repeat period */

/* Call every main loop pass with the current obstacle level and drive mode. */
void alert_set(uint8_t level, uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif /* ALERT_H */
