/**
  ******************************************************************************
  * @file    motor.h
  * @brief   L298N dual H-bridge driver for a 4WD skid-steer base.
  *          Channel A = IN1/IN2 + ENA, channel B = IN3/IN4 + ENB.
  *          Which robot side each channel drives is set by MOTOR_SWAP_SIDES
  *          in motor.c (current wiring: A = right, B = left).
  *          left/right in this API always mean the robot's real sides.
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

/* Invert a whole real robot side (1 = swap forward/backward). */
extern volatile uint8_t MOTOR_L_INVERT;
extern volatile uint8_t MOTOR_R_INVERT;
/* Minimum duty in %; when > 0, 1..100 % is mapped to deadband..100 %. */
extern volatile uint8_t motor_deadband;

void motor_init(TIM_HandleTypeDef *htim, uint32_t ch_a, uint32_t ch_b);  /* ENA, ENB channels */
void motor_set(int left, int right);       /* -100..100 %, negative = reverse */
void motor_get_target(int *left, int *right);
void motor_set_limit(int pct);             /* cap |left|,|right| to pct (100 = none) */
int  motor_get_limit(void);
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
