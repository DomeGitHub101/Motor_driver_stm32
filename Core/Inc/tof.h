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
#define TOF_POLL_MS         10U   /* I2C status check period (sensor: ~33 ms/sample) */
#define TOF_TIMEOUT_MS     300U   /* no good sample this long = sensor error */
#define TOF_RESET_CHECK_MS  70U   /* quiet this long (2 samples): check for a reset */
#define TOF_RETRY_MS       500U   /* re-init period while in error */
#define TOF_MEDIAN_N         5U   /* median filter length (~33 ms per sample) */
#define TOF_INIT_ATTEMPTS    2U   /* XSHUT power-cycle + init tries per start */
#define TOF_BOOT_MS         20U   /* wait after XSHUT high (datasheet: 1.2 ms) */
/* I2C1 TIMINGR: CubeMX's 100 kHz value (I2CCLK = PCLK1 = 54 MHz); with the
 * DNF 15 filter it runs at ~97 kHz. 50 kHz (0x70823C46) made a re-init take
 * ~110 ms (about 130 register writes), too slow to hide a sensor reset;
 * 20 kHz (0xF0424E5B) made the VL53L0X stop answering (err 10). */
#define TOF_I2C_TIMING  0x20404768U

void     tof_init(void);          /* blocking, call once at boot */
void     tof_poll(void);          /* non-blocking, call from the main loop */
uint16_t tof_read_mm(void);       /* latest distance in mm */
bool     tof_ok(void);            /* false = sensor error (treat as level 3) */
uint32_t tof_glitch_count(void);  /* I2C failures absorbed without an error */
uint32_t tof_reset_count(void);   /* sensor self-resets re-inited on the fly */
uint16_t tof_raw_mm(void);        /* latest sample before the median filter */
void     tof_set_retry_ms(uint32_t ms);  /* re-init period while in error */

/* Test hook: force tof_read_mm() to return mm (< 0 = use the real sensor). */
void     tof_set_override(int mm);
int      tof_get_override(void);

#ifdef __cplusplus
}
#endif

#endif /* TOF_H */
