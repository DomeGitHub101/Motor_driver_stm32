/**
  ******************************************************************************
  * @file    alert.h
  * @brief   Obstacle voice alert through an I2S amplifier (I2S2 + DMA).
  ******************************************************************************
  */
#ifndef ALERT_H
#define ALERT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define ALERT_REPEAT_MS       3000U   /* level 3 repeat period */
#define ALERT_VOLUME_PERCENT    50    /* 0..100 */

void alert_init(I2S_HandleTypeDef *hi2s);   /* starts silent circular DMA */

/* Call every main loop pass with the current obstacle level and drive mode. */
void alert_set(uint8_t level, uint8_t mode);

void alert_i2s_half_cplt(void);   /* call from HAL_I2S_TxHalfCpltCallback */
void alert_i2s_cplt(void);        /* call from HAL_I2S_TxCpltCallback */

#ifdef __cplusplus
}
#endif

#endif /* ALERT_H */
