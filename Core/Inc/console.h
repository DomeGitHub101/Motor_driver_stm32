/**
  ******************************************************************************
  * @file    console.h
  * @brief   Serial test console (Tera Term) for the motor driver.
  ******************************************************************************
  */
#ifndef CONSOLE_H
#define CONSOLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

#define CONSOLE_MANUAL_HOLD_MS  2000U  /* console motion commands override the link this long */

void console_init(UART_HandleTypeDef *huart);
void console_poll(void);      /* call from the main loop */
void console_rx_cplt(void);   /* call from HAL_UART_RxCpltCallback */
void console_rx_error(void);  /* call from HAL_UART_ErrorCallback */

bool console_manual_active(void);    /* motion command typed recently or test running */
void console_link_takeover(void);    /* valid $M received: abort the test sequence */

#ifdef __cplusplus
}
#endif

#endif /* CONSOLE_H */
