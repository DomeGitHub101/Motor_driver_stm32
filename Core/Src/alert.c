/**
  ******************************************************************************
  * @file    alert.c
  * @brief   Obstacle alert. For now only prints what would be played;
  *          replace alert_play() with the real buzzer / speaker driver.
  *          Non-blocking: called from the main loop, no delays.
  ******************************************************************************
  */
#include "alert.h"
#include <stdio.h>

#define ALERT_MODE_DOCKING  2U

static uint8_t  alert_level;
static uint32_t alert_last_play;

static void alert_play(uint8_t level)
{
  /* TODO: drive the real buzzer / speaker here */
  printf("\r\nALERT L%u: beep + speak \"กรุณาหลบหน่อยครับ\" (please make way)\r\n",
         (unsigned)level);
}

void alert_set(uint8_t level, uint8_t mode)
{
  /* Docking at the table: the ToF sees the table, stay silent */
  uint8_t eff = (mode == ALERT_MODE_DOCKING) ? 0U : level;
  uint32_t now = HAL_GetTick();

  if (eff != alert_level)
  {
    alert_level = eff;
    if (eff >= 2U)
    {
      alert_play(eff);   /* once on entering level 2, first of the level 3 repeats */
      alert_last_play = now;
    }
  }
  else if ((eff >= 3U) && ((now - alert_last_play) >= ALERT_REPEAT_MS))
  {
    alert_play(eff);
    alert_last_play = now;
  }
}
