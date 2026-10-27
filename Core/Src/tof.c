/**
  ******************************************************************************
  * @file    tof.c
  * @brief   Front distance sensor: one VL53L0X in continuous mode.
  *          tof_poll() only checks the result flag (one short I2C read),
  *          so the main loop is never blocked waiting for a measurement.
  ******************************************************************************
  */
#include "tof.h"
#include "vl53l0x_simple.h"
#include "i2c.h"
#include <stdio.h>

#define TOF_ERR_TIMEOUT  100

static tof0_t   tof_dev = { TOF0_DEFAULT_ADDR, 0 };
static bool     tof_good;
static int      tof_err;
static uint16_t tof_mm = TOF_NO_OBJECT_MM;
static uint32_t tof_last_sample;
static uint32_t tof_last_poll;
static uint32_t tof_last_try;
static uint32_t tof_retry_ms = TOF_RETRY_MS;
static uint32_t retry_fails;   /* consecutive failed re-inits */
static int      tof_override = -1;
static uint32_t tof_glitches;  /* failures absorbed since boot */
static uint16_t tof_raw = TOF_NO_OBJECT_MM;  /* latest unfiltered sample (debug) */
static uint16_t tof_sensor_mm = TOF_NO_OBJECT_MM; /* latest sample before clamping */
static uint32_t fails_since_good;     /* I2C failures since the last sample */
static uint32_t notready_since_good;  /* "no new sample yet" polls since then */
static bool     print_next_sample;    /* report the first sample after recovery */
static bool     reinit_done;          /* reset check already done in this gap */
static uint32_t tof_resets;           /* sensor self-resets fixed without an error */
static uint16_t med_buf[TOF_MEDIAN_N];
static uint32_t med_count;     /* samples stored, saturates at TOF_MEDIAN_N */
static uint32_t med_pos;

/* Median of the last TOF_MEDIAN_N samples (fewer right after a (re)start):
 * drops single-sample spikes, follows a real object within 3 samples. */
static uint16_t median_push(uint16_t v)
{
  uint16_t s[TOF_MEDIAN_N];
  uint32_t i, j, n;

  med_buf[med_pos] = v;
  med_pos = (med_pos + 1U) % TOF_MEDIAN_N;
  if (med_count < TOF_MEDIAN_N)
  {
    med_count++;
  }

  n = med_count;
  for (i = 0U; i < n; i++)
  {
    s[i] = med_buf[i];
  }
  for (i = 1U; i < n; i++)   /* insertion sort, n <= 5 */
  {
    uint16_t t = s[i];
    for (j = i; (j > 0U) && (s[j - 1U] > t); j--)
    {
      s[j] = s[j - 1U];
    }
    s[j] = t;
  }
  return s[n / 2U];
}

/* TOF_I2C_TIMING plus the analog filter and the longest digital filter
 * (15 x tI2CCLK): filtered edges make short motor spikes less likely to
 * corrupt a transfer. */
static void i2c_configure(void)
{
  hi2c1.Init.Timing = TOF_I2C_TIMING;
  (void)HAL_I2C_Init(&hi2c1);
  (void)HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
  (void)HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 15U);
}

/* Free a bus a slave may hold low after a corrupted transfer: re-init I2C1,
 * clocking SCL 9 times and sending a STOP while the pins are plain GPIO. */
static void i2c_bus_recover(void)
{
  GPIO_InitTypeDef g = {0};
  int i;

  (void)HAL_I2C_DeInit(&hi2c1);

  g.Pin = GPIO_PIN_8 | GPIO_PIN_9;   /* PB8 = SCL, PB9 = SDA */
  g.Mode = GPIO_MODE_OUTPUT_OD;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
  HAL_GPIO_Init(GPIOB, &g);

  for (i = 0; i < 9; i++)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    for (volatile int d = 0; d < 500; d++) { }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    for (volatile int d = 0; d < 500; d++) { }
  }
  /* STOP: SDA low -> high while SCL is high */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
  for (volatile int d = 0; d < 500; d++) { }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

  i2c_configure();   /* HAL_I2C_Init's MspInit puts PB8/PB9 back to AF4 */
}

/* Init the sensor and start continuous ranging, 0 = OK */
static int sensor_init(void)
{
  int e;

  tof_dev.addr = TOF0_DEFAULT_ADDR;
  e = tof0_init(&tof_dev);
  if (e == 0)
  {
    e = tof0_start_continuous(&tof_dev);
  }
  return e;
}

/* Power-cycle the sensor with XSHUT, init it and start continuous ranging.
 * Tries TOF_INIT_ATTEMPTS times: calibration often fails once on a noisy supply. */
static bool tof_start(void)
{
  int e = -1;
  uint32_t attempt;

  for (attempt = 0U; (attempt < TOF_INIT_ATTEMPTS) && (e != 0); attempt++)
  {
    if (attempt > 0U)
    {
      i2c_bus_recover();
    }
    HAL_GPIO_WritePin(XSHUT_GPIO_Port, XSHUT_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(XSHUT_GPIO_Port, XSHUT_Pin, GPIO_PIN_SET);
    HAL_Delay(TOF_BOOT_MS);
    e = sensor_init();
  }
  tof_err = e;
  tof_good = (e == 0);
  med_count = 0U;
  med_pos = 0U;
  fails_since_good = 0U;
  notready_since_good = 0U;
  reinit_done = false;
  tof_last_sample = HAL_GetTick();
  return tof_good;
}

/* Called when the sensor has been quiet for TOF_RESET_CHECK_MS although I2C
 * works. SEQUENCE_CONFIG is 0xE8 after our init; any other value means the
 * sensor reset itself (supply dip) and stopped ranging: re-init it now so it
 * is back before TOF_TIMEOUT_MS and no error is declared. Once per gap. */
static void check_sensor_reset(void)
{
  uint8_t seq;
  uint32_t t0;
  int e;

  reinit_done = true;
  if ((tof0_read_reg(&tof_dev, 0x01U, &seq) != 0) || (seq == 0xE8U))
  {
    return;
  }
  t0 = HAL_GetTick();
  e = sensor_init();   /* one try: a failure is left to the error/retry path */
  tof_resets++;
  printf("\r\nTOF: sensor reset (seq_cfg=0x%02X), re-init %s (err %d) in %lu ms, %lu resets\r\n",
         (unsigned)seq, (e == 0) ? "ok" : "failed", e,
         (unsigned long)(HAL_GetTick() - t0), (unsigned long)tof_resets);
}

/* What the sensor gave before the error, for debugging the cause */
static void print_error(uint32_t now)
{
  uint32_t i;

  printf("\r\nTOF: read error %d (%s), treating as level 3\r\n", tof_err,
         (tof_err == TOF_ERR_TIMEOUT) ? "no new sample" : "I2C transfer failed");
  printf("  last sample %lu ms ago: sensor=%u mm raw=%u dist=%u\r\n",
         (unsigned long)(now - tof_last_sample), (unsigned)tof_sensor_mm,
         (unsigned)tof_raw, (unsigned)tof_mm);
  printf("  since then: i2c_fail=%lu not_ready=%lu, total glitches=%lu\r\n",
         (unsigned long)fails_since_good, (unsigned long)notready_since_good,
         (unsigned long)tof_glitches);
  printf("  last %lu samples (oldest first):", (unsigned long)med_count);
  for (i = 0U; i < med_count; i++)
  {
    uint32_t k = (med_pos + TOF_MEDIAN_N - med_count + i) % TOF_MEDIAN_N;
    printf(" %u", (unsigned)med_buf[k]);
  }
  printf("\r\n");
  if (tof_err == TOF_ERR_TIMEOUT)
  {
    /* SEQUENCE_CONFIG is 0xE8 after our init: anything else = sensor reset */
    uint8_t seq = 0U, start = 0U;
    int e = tof0_read_reg(&tof_dev, 0x01U, &seq) | tof0_read_reg(&tof_dev, 0x00U, &start);
    printf("  regs: seq_cfg=0x%02X (0xE8 = configured) sysrange=0x%02X%s\r\n",
           (unsigned)seq, (unsigned)start, e ? " (read failed)" : "");
  }
}

void tof_init(void)
{
  uint32_t t0;

  i2c_configure();
  tof_last_try = HAL_GetTick();
  t0 = HAL_GetTick();
  if (tof_start())
  {
    printf("TOF: VL53L0X ready (init %lu ms)\r\n", (unsigned long)(HAL_GetTick() - t0));
  }
  else
  {
    printf("TOF: init failed (err %d), wheels blocked until it works\r\n", tof_err);
  }
}

void tof_poll(void)
{
  uint32_t now = HAL_GetTick();
  uint16_t mm;
  int r;

  if ((now - tof_last_poll) < TOF_POLL_MS)
  {
    return;
  }
  tof_last_poll = now;

  if (!tof_good)
  {
    if ((now - tof_last_try) >= tof_retry_ms)
    {
      int prev_err = tof_err;

      tof_last_try = now;
      i2c_bus_recover();
      if (tof_start())
      {
        printf("\r\nTOF: recovered\r\n");
        print_next_sample = true;
        retry_fails = 0U;
      }
      else if ((tof_err != prev_err) || ((++retry_fails % 10U) == 0U))
      {
        printf("\r\nTOF: re-init failed (err %d, %lu tries)\r\n", tof_err,
               (unsigned long)(retry_fails + 1U));
      }
      now = HAL_GetTick();   /* re-init blocks; restart the poll period */
      tof_last_poll = now;
    }
    return;
  }

  r = tof0_poll_mm(&tof_dev, &mm);
  if ((r == 0) && (mm == 0U))
  {
    /* 0 mm is not a real range (seen right before noise errors): drop it */
    tof_glitches++;
    fails_since_good++;
  }
  else if (r == 0)
  {
    tof_sensor_mm = mm;
    tof_raw = (mm > TOF_NO_OBJECT_MM) ? (uint16_t)TOF_NO_OBJECT_MM : mm;
    tof_mm = median_push(tof_raw);
    tof_last_sample = now;
    fails_since_good = 0U;
    notready_since_good = 0U;
    reinit_done = false;
    if (print_next_sample)
    {
      print_next_sample = false;
      printf("TOF: first sample sensor=%u mm raw=%u dist=%u\r\n",
             (unsigned)tof_sensor_mm, (unsigned)tof_raw, (unsigned)tof_mm);
    }
    return;
  }
  else if (r > 0)
  {
    /* Failed transfers are motor noise most of the time: free the bus and
     * keep the last distance; only TOF_TIMEOUT_MS without a sample is an error. */
    tof_glitches++;
    fails_since_good++;
    i2c_bus_recover();
  }
  else
  {
    notready_since_good++;
    if (((now - tof_last_sample) >= TOF_RESET_CHECK_MS) && !reinit_done)
    {
      check_sensor_reset();
      now = HAL_GetTick();
    }
  }

  if ((now - tof_last_sample) > TOF_TIMEOUT_MS)
  {
    tof_good = false;
    tof_err = (r > 0) ? r : TOF_ERR_TIMEOUT;
    /* I2C answers but no samples: the sensor most likely reset itself
     * (supply dip) and stopped ranging, so re-init on the next poll. */
    tof_last_try = (tof_err == TOF_ERR_TIMEOUT) ? (now - tof_retry_ms) : now;
    print_error(now);
  }
}

uint16_t tof_read_mm(void)
{
  if (tof_override >= 0)
  {
    return (uint16_t)tof_override;
  }
  return tof_good ? tof_mm : 0U;
}

bool tof_ok(void)
{
  return (tof_override >= 0) || tof_good;
}

uint32_t tof_glitch_count(void)
{
  return tof_glitches;
}

uint32_t tof_reset_count(void)
{
  return tof_resets;
}

void tof_set_retry_ms(uint32_t ms)
{
  tof_retry_ms = ms;
}

uint16_t tof_raw_mm(void)
{
  return tof_raw;
}

void tof_set_override(int mm)
{
  if (mm > 65535)
  {
    mm = 65535;
  }
  tof_override = mm;
}

int tof_get_override(void)
{
  return tof_override;
}
