/**
  ******************************************************************************
  * @file    link.h
  * @brief   UART link to the upper board (USART2).
  *          RX: $M,left,right,mode*XX   TX: $O,level,dist_mm*XX
  ******************************************************************************
  */
#ifndef LINK_H
#define LINK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

/* Obstacle level thresholds in mm (level rises when closer than these) */
#define LINK_LEVEL1_MM      1000U   /* 600..1000 -> 1 */
#define LINK_LEVEL2_MM       600U   /* 300..600  -> 2 */
#define LINK_LEVEL3_MM       300U   /* < 300     -> 3 */
#define LINK_HYST_MM          50U   /* extra distance needed to drop a level */
#define LINK_LEVEL_DROP_MS   500U   /* lower level must hold this long to drop */

#define LINK_LEVEL1_MAX_PCT   30    /* forward limit per wheel at level 1 */

/* ToF error handling ("safe" console command):
 *   strict: a sensor error counts as level 3 (e-stop)
 *   slow:   keep driving, every wheel limited to LINK_TOF_FAIL_MAX_PCT */
#define LINK_TOF_FAIL_SLOW_DEFAULT  1     /* 1 = slow at boot, 0 = strict */
#define LINK_TOF_FAIL_MAX_PCT      25
#define LINK_TOF_RETRY_SLOW_MS   2000U    /* sensor re-init period while driving slow */
#define LINK_TX_PERIOD_MS    200U   /* $O period */
#define LINK_TOF_PERIOD_MS    50U   /* ToF sampling period */
#define LINK_LINE_MAX         47U   /* longer lines are dropped */

void link_init(UART_HandleTypeDef *huart);
void link_poll(void);       /* call from the main loop */
void link_rx_cplt(void);    /* call from HAL_UART_RxCpltCallback */
void link_rx_error(void);   /* call from HAL_UART_ErrorCallback */

void    link_debug_toggle(void);
bool    link_debug_enabled(void);
uint8_t link_obstacle_level(void);

void    link_set_tof_fail_slow(bool slow);  /* true = slow, false = strict */
bool    link_tof_fail_slow(void);
bool    link_slow_active(void);             /* sensor failed and driving slow now */

#ifdef __cplusplus
}
#endif

#endif /* LINK_H */
