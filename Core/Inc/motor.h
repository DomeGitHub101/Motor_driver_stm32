/**
  ******************************************************************************
  * @file    motor.h
  * @brief   L298N dual H-bridge driver for a 4WD skid-steer base.
  *          Left side  = channel A (IN1/IN2 + ENA)
  *          Right side = channel B (IN3/IN4 + ENB)
  ******************************************************************************
  */
#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stddef.h>

#define MOTOR_TICK_MS       20U   /* motor_tick() call period */
#define MOTOR_RAMP_STEP     5     /* max change in % per tick */
#define MOTOR_WATCHDOG_MS   500U  /* timeout without set/feed */

/* Invert a whole side (1 = swap forward/backward). */
extern volatile uint8_t MOTOR_L_INVERT;
extern volatile uint8_t MOTOR_R_INVERT;
/* Minimum duty in %; when > 0, 1..100 % is mapped to deadband..100 %. */
extern volatile uint8_t motor_deadband;

void motor_init(TIM_HandleTypeDef *htim, uint32_t ch_left, uint32_t ch_right);
void motor_set(int left, int right);       /* -100..100 %, negative = reverse */
void motor_get_target(int *left, int *right);
void motor_tick(void);                     /* call from 20 ms timer ISR */
void motor_estop(void);                    /* immediate stop, no ramp */

void motor_feed(void);
void motor_watchdog_enable(bool enable);
bool motor_watchdog_is_enabled(void);
bool motor_watchdog_tripped(void);         /* true once after a watchdog stop */

void motor_status_str(char *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
