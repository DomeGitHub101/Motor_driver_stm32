/**
  ******************************************************************************
  * @file    tof.c
  * @brief   Front distance sensor. Stub until the ToF model is chosen:
  *          replace tof_init() / tof_read_mm() with the real driver.
  ******************************************************************************
  */
#include "tof.h"

static int tof_override = -1;

void tof_init(void)
{
  /* TODO: init the real ToF sensor here */
}

uint16_t tof_read_mm(void)
{
  if (tof_override >= 0)
  {
    return (uint16_t)tof_override;
  }
  /* TODO: read the real ToF sensor here */
  return TOF_NO_OBJECT_MM;
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
