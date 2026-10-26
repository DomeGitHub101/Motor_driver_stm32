/**
  ******************************************************************************
  * @file    alert.c
  * @brief   Obstacle voice alert. 16 kHz 16-bit mono PCM clips are streamed
  *          to I2S2 through a circular DMA buffer refilled half by half in
  *          the DMA callbacks. alert_set() only picks what to play, so it
  *          never blocks the main loop.
  *
  *   level 1: voice2000 "ขอโทษครับ กรุณาหลบหน่อยครับ"  once on entering
  *   level 2: voice1000 "กรุณาหลบหน่อยครับ"            once on entering
  *   level 3: voice500  "หลบไปดิ้"                     every ALERT_REPEAT_MS
  *   mode 2 (docking): silent at every level
  ******************************************************************************
  */
#include "alert.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "voice500.h"
#include "voice1000.h"
#include "voice2000.h"

#define ALERT_MODE_DOCKING  2U
#define AUDIO_FRAMES        512U   /* stereo frames per half buffer (32 ms) */

static I2S_HandleTypeDef *alert_hi2s;
static int16_t dma_buf[AUDIO_FRAMES * 2U * 2U] __attribute__((aligned(32)));

static const uint8_t * volatile cur_data;
static volatile uint32_t cur_samples;
static volatile uint32_t audio_pos;
static volatile uint8_t  tail_halves;
static volatile bool     voice_active;

static uint8_t  alert_level;
static uint32_t alert_last_play;

/* Runs in the DMA ISR: fill one half buffer with the next samples or silence */
static void fill_half(int16_t *dst)
{
  /* clip fully queued: let the 2 halves in flight play out, then go idle */
  if (voice_active && (audio_pos >= cur_samples))
  {
    if (++tail_halves >= 2U)
    {
      voice_active = false;
    }
  }
  for (uint32_t i = 0U; i < AUDIO_FRAMES; i++)
  {
    int16_t s = 0;

    if (voice_active && (audio_pos < cur_samples))
    {
      memcpy(&s, cur_data + audio_pos * 2U, 2U);
      s = (int16_t)((s * ALERT_VOLUME_PERCENT) / 100);
      audio_pos++;
    }
    dst[2U * i]      = s;   /* left */
    dst[2U * i + 1U] = s;   /* right */
  }
  /* no-op while the D-cache is off, required if it is enabled later */
  SCB_CleanDCache_by_Addr((uint32_t *)dst, (int32_t)(AUDIO_FRAMES * 2U * sizeof(int16_t)));
}

void alert_i2s_half_cplt(void)
{
  fill_half(&dma_buf[0]);
}

void alert_i2s_cplt(void)
{
  fill_half(&dma_buf[AUDIO_FRAMES * 2U]);
}

void alert_init(I2S_HandleTypeDef *hi2s)
{
  alert_hi2s = hi2s;
  fill_half(&dma_buf[0]);
  fill_half(&dma_buf[AUDIO_FRAMES * 2U]);
  if (HAL_I2S_Transmit_DMA(alert_hi2s, (uint16_t *)dma_buf, (uint16_t)(AUDIO_FRAMES * 4U)) != HAL_OK)
  {
    printf("ALERT: I2S start failed, no sound\r\n");
  }
}

static void alert_play(uint8_t level)
{
  const uint8_t *data;
  uint32_t len;
  const char *text;
  uint32_t primask;

  switch (level)
  {
    case 1:  data = voice2000_data; len = voice2000_len; text = "ขอโทษครับ กรุณาหลบหน่อยครับ"; break;
    case 2:  data = voice1000_data; len = voice1000_len; text = "กรุณาหลบหน่อยครับ"; break;
    default: data = voice500_data;  len = voice500_len;  text = "หลบไปดิ้"; break;
  }

  /* restart cleanly even if another clip is playing */
  primask = __get_PRIMASK();
  __disable_irq();
  cur_data = data;
  cur_samples = len / 2U;
  audio_pos = 0U;
  tail_halves = 0U;
  voice_active = true;
  __set_PRIMASK(primask);

  printf("\r\nALERT L%u: \"%s\"\r\n", (unsigned)level, text);
}

void alert_set(uint8_t level, uint8_t mode)
{
  /* Docking at the table: the ToF sees the table, stay silent */
  uint8_t eff = (mode == ALERT_MODE_DOCKING) ? 0U : level;
  uint32_t now = HAL_GetTick();

  if (eff > alert_level)
  {
    alert_play(eff);   /* speak only when the obstacle gets closer */
    alert_last_play = now;
  }
  else if ((eff >= 3U) && ((now - alert_last_play) >= ALERT_REPEAT_MS))
  {
    alert_play(eff);
    alert_last_play = now;
  }
  alert_level = eff;
}
