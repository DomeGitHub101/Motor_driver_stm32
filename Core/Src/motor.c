/**
  ******************************************************************************
  * @file    motor.c
  * @brief   L298N dual H-bridge driver with ramping, deadband and watchdog.
  ******************************************************************************
  */
#include "motor.h"
#include <stdio.h>

volatile uint8_t MOTOR_L_INVERT = 0;
volatile uint8_t MOTOR_R_INVERT = 0;
volatile uint8_t motor_deadband = 0;

static TIM_HandleTypeDef *motor_htim;
static uint32_t motor_ch_l;
static uint32_t motor_ch_r;

/* Logical values (before invert), in % */
static volatile int16_t target_l, target_r;
static volatile int16_t actual_l, actual_r;
/* Sign of the last value driven to the bridge (after invert) */
static volatile int8_t phys_sign_l, phys_sign_r;

static volatile bool     wd_enabled;
static volatile bool     wd_expired;
static volatile bool     wd_tripped;
static volatile uint32_t wd_last_feed;

static int clamp_pct(int v)
{
  if (v > 100)  return 100;
  if (v < -100) return -100;
  return v;
}

static uint32_t pct_to_ccr(int pct)
{
  uint32_t mag = (uint32_t)((pct < 0) ? -pct : pct);
  uint32_t db  = motor_deadband;

  if (mag == 0U)
  {
    return 0U;
  }
  if (db > 0U)
  {
    /* Map 1..100 % -> db..100 % */
    mag = db + ((mag - 1U) * (100U - db) + 49U) / 99U;
  }
  /* 100 % -> ARR+1, which keeps PWM mode 1 output high for the whole period */
  return mag * (__HAL_TIM_GET_AUTORELOAD(motor_htim) + 1U) / 100U;
}

/* Drive one side of the bridge from its logical value. Must not run concurrently
 * with itself for the same side (called from the tick ISR or with IRQs off). */
static void apply_side(volatile int16_t *actual, volatile int8_t *phys_sign, uint8_t invert,
                       uint32_t ch, GPIO_TypeDef *port_a, uint16_t pin_a,
                       GPIO_TypeDef *port_b, uint16_t pin_b)
{
  int phys = invert ? -*actual : *actual;
  int8_t sign = (phys > 0) ? 1 : ((phys < 0) ? -1 : 0);

  /* Invert toggled while moving would reverse without passing 0: stop first */
  if ((sign != 0) && (*phys_sign != 0) && (sign != *phys_sign))
  {
    *actual = 0;
    phys = 0;
    sign = 0;
  }

  if (sign == 0)
  {
    __HAL_TIM_SET_COMPARE(motor_htim, ch, 0U);
    HAL_GPIO_WritePin(port_a, pin_a, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(port_b, pin_b, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(port_a, pin_a, (sign > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(port_b, pin_b, (sign < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(motor_htim, ch, pct_to_ccr(phys));
  }
  *phys_sign = sign;
}

static void apply_outputs(void)
{
  apply_side(&actual_l, &phys_sign_l, MOTOR_L_INVERT, motor_ch_l,
             IN1_GPIO_Port, IN1_Pin, IN2_GPIO_Port, IN2_Pin);
  apply_side(&actual_r, &phys_sign_r, MOTOR_R_INVERT, motor_ch_r,
             IN3_GPIO_Port, IN3_Pin, IN4_GPIO_Port, IN4_Pin);
}

/* Move actual toward target by at most MOTOR_RAMP_STEP, never crossing 0 in one step */
static int16_t ramp(int16_t actual, int16_t target)
{
  int16_t goal = target;

  if (((actual > 0) && (target < 0)) || ((actual < 0) && (target > 0)))
  {
    goal = 0;
  }
  if (goal > actual)
  {
    return (goal - actual > MOTOR_RAMP_STEP) ? (int16_t)(actual + MOTOR_RAMP_STEP) : goal;
  }
  if (goal < actual)
  {
    return (actual - goal > MOTOR_RAMP_STEP) ? (int16_t)(actual - MOTOR_RAMP_STEP) : goal;
  }
  return actual;
}

void motor_init(TIM_HandleTypeDef *htim, uint32_t ch_left, uint32_t ch_right)
{
  motor_htim = htim;
  motor_ch_l = ch_left;
  motor_ch_r = ch_right;

  target_l = target_r = 0;
  actual_l = actual_r = 0;
  phys_sign_l = phys_sign_r = 0;
  wd_enabled = false;
  wd_expired = false;
  wd_tripped = false;
  wd_last_feed = HAL_GetTick();

  apply_outputs();
  HAL_TIM_PWM_Start(motor_htim, motor_ch_l);
  HAL_TIM_PWM_Start(motor_htim, motor_ch_r);
}

void motor_set(int left, int right)
{
  target_l = (int16_t)clamp_pct(left);
  target_r = (int16_t)clamp_pct(right);
  motor_feed();
}

void motor_get_target(int *left, int *right)
{
  if (left != NULL)  *left = target_l;
  if (right != NULL) *right = target_r;
}

void motor_feed(void)
{
  wd_last_feed = HAL_GetTick();
  wd_expired = false;
}

void motor_watchdog_enable(bool enable)
{
  motor_feed();
  wd_tripped = false;
  wd_enabled = enable;
}

bool motor_watchdog_is_enabled(void)
{
  return wd_enabled;
}

bool motor_watchdog_tripped(void)
{
  if (wd_tripped)
  {
    wd_tripped = false;
    return true;
  }
  return false;
}

void motor_tick(void)
{
  if (motor_htim == NULL)
  {
    return;
  }

  if (wd_enabled && !wd_expired && ((HAL_GetTick() - wd_last_feed) > MOTOR_WATCHDOG_MS))
  {
    wd_expired = true;
    if ((target_l != 0) || (target_r != 0) || (actual_l != 0) || (actual_r != 0))
    {
      target_l = 0;
      target_r = 0;
      wd_tripped = true;
    }
  }

  actual_l = ramp(actual_l, target_l);
  actual_r = ramp(actual_r, target_r);
  apply_outputs();
}

void motor_estop(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  target_l = target_r = 0;
  actual_l = actual_r = 0;
  if (motor_htim != NULL)
  {
    apply_outputs();
  }
  __set_PRIMASK(primask);
}

void motor_status_str(char *buf, size_t len)
{
  int l = target_l;
  int r = target_r;
  int pl = MOTOR_L_INVERT ? -l : l;
  int pr = MOTOR_R_INVERT ? -r : r;

  if ((buf == NULL) || (len == 0U))
  {
    return;
  }
  snprintf(buf, len, "L=%+d%% R=%+d%% (IN1=%d IN2=%d IN3=%d IN4=%d)",
           l, r, pl > 0, pl < 0, pr > 0, pr < 0);
}
