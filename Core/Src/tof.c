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
#include <stdio.h>

#define TOF_ERR_TIMEOUT  100

static tof0_t   tof_dev = { TOF0_DEFAULT_ADDR, 0 };
static bool     tof_good;
static int      tof_err;
static uint16_t tof_mm = TOF_NO_OBJECT_MM;
static uint32_t tof_last_sample;
static uint32_t tof_last_poll;
static uint32_t tof_last_try;
static int      tof_override = -1;

/* Power-cycle the sensor with XSHUT, init it and start continuous ranging */
static bool tof_start(void)
{
  int e;

  HAL_GPIO_WritePin(XSHUT_GPIO_Port, XSHUT_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(XSHUT_GPIO_Port, XSHUT_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  tof_dev.addr = TOF0_DEFAULT_ADDR;
  e = tof0_init(&tof_dev);
  if (e == 0)
  {
    e = tof0_start_continuous(&tof_dev);
  }
  tof_err = e;
  tof_good = (e == 0);
  tof_last_sample = HAL_GetTick();
  return tof_good;
}

void tof_init(void)
{
  tof_last_try = HAL_GetTick();
  if (tof_start())
  {
    printf("TOF: VL53L0X ready\r\n");
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
    if ((now - tof_last_try) >= TOF_RETRY_MS)
    {
      tof_last_try = now;
      if (tof_start())
      {
        printf("\r\nTOF: recovered\r\n");
      }
    }
    return;
  }

  r = tof0_poll_mm(&tof_dev, &mm);
  if (r == 0)
  {
    tof_mm = (mm > TOF_NO_OBJECT_MM) ? (uint16_t)TOF_NO_OBJECT_MM : mm;
    tof_last_sample = now;
  }
  else if ((r > 0) || ((now - tof_last_sample) > TOF_TIMEOUT_MS))
  {
    tof_good = false;
    tof_err = (r > 0) ? r : TOF_ERR_TIMEOUT;
    tof_last_try = now;
    printf("\r\nTOF: read error %d, treating as level 3\r\n", tof_err);
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
