/**
  ******************************************************************************
  * @file    link.c
  * @brief   UART link to the upper board on USART2.
  *
  *   Frame: $<body>*XX\r\n, XX = XOR of all chars between '$' and '*',
  *          2 upper-case hex digits. Bad checksum / format / > 47 chars = dropped.
  *   RX:    $M,left,right,mode*XX   (-100..100 %, mode 0 stop, 1 run, 2 docking)
  *   TX:    $O,level,dist_mm*XX     every 200 ms and on every level change
  *
  *   Drive priority:
  *     level 3            -> motor_estop(), ignores everything (console too)
  *     ToF sensor error   -> "safe strict": same as level 3
  *                           "safe slow":   all wheels capped at LINK_TOF_FAIL_MAX_PCT
  *                           (both report $O,3,0 to the upper board)
  *     level 2, mode != 2 -> no forward (left + right > 0 -> 0, 0)
  *     level 1, mode != 2 -> forward limited to LINK_LEVEL1_MAX_PCT per wheel
  *     mode 0             -> motor_set(0, 0)
  *     otherwise          -> motor_set(left, right)
  *   Console motion commands override $M for 2 s (see console_manual_active()).
  ******************************************************************************
  */
#include "link.h"
#include "motor.h"
#include "tof.h"
#include "alert.h"
#include "console.h"
#include <stdio.h>
#include <string.h>

#define RX_RING_SIZE      128U   /* power of two */
#define DEBUG_PERIOD_MS   500U   /* "link" print rate limit */
#define MODE_STOP         0
#define MODE_DOCKING      2

static UART_HandleTypeDef *link_huart;
static uint8_t rx_byte;
static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head;   /* written by ISR */
static volatile uint16_t rx_tail;   /* written by main loop */

static char     line_buf[LINK_LINE_MAX + 1U];
static uint16_t line_len;
static bool     line_active;
static bool     line_overflow;

/* Last valid $M */
static bool     cmd_valid;
static int      cmd_left, cmd_right, cmd_mode;
static uint32_t cmd_tick;
static char     cmd_text[LINK_LINE_MAX + 1U];

/* Values actually given to motor_set() by the link */
static int      out_left, out_right;

static uint8_t  obst_level;   /* reported in $O (3 on a sensor error) */
static uint16_t obst_dist;
static uint32_t tof_tick;

static bool     fail_slow = (LINK_TOF_FAIL_SLOW_DEFAULT != 0);
static bool     slow_active;  /* sensor error while fail_slow */

static char     tx_buf[24];
static bool     tx_pending;
static uint32_t tx_tick;

static bool     debug_on;
static uint32_t debug_tick;
static uint32_t rx_ok, rx_bad;

/* ---------------------------------------------------------------- RX (ISR) */

static void rx_arm(void)
{
  (void)HAL_UART_Receive_IT(link_huart, &rx_byte, 1U);
}

void link_rx_cplt(void)
{
  uint16_t next = (uint16_t)((rx_head + 1U) & (RX_RING_SIZE - 1U));

  if (next != rx_tail)   /* drop byte when full */
  {
    rx_ring[rx_head] = rx_byte;
    rx_head = next;
  }
  rx_arm();
}

void link_rx_error(void)
{
  __HAL_UART_CLEAR_FLAG(link_huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
  rx_arm();
}

static bool rx_get(char *c)
{
  if (rx_tail == rx_head)
  {
    return false;
  }
  *c = (char)rx_ring[rx_tail];
  rx_tail = (uint16_t)((rx_tail + 1U) & (RX_RING_SIZE - 1U));
  return true;
}

/* ------------------------------------------------------------ frame parse */

static uint8_t xor_sum(const char *s, size_t n)
{
  uint8_t cs = 0U;

  while (n-- > 0U)
  {
    cs ^= (uint8_t)*s++;
  }
  return cs;
}

static int hex_upper(char c)
{
  if ((c >= '0') && (c <= '9')) return c - '0';
  if ((c >= 'A') && (c <= 'F')) return c - 'A' + 10;
  return -1;
}

/* Parse [-]digits (max 3) followed by term; advances *pp past term. */
static bool parse_field(const char **pp, char term, int *out)
{
  const char *p = *pp;
  bool neg = false;
  int v = 0;
  int n = 0;

  if (*p == '-')
  {
    neg = true;
    p++;
  }
  while ((*p >= '0') && (*p <= '9'))
  {
    if (++n > 3)
    {
      return false;
    }
    v = v * 10 + (*p - '0');
    p++;
  }
  if ((n == 0) || (*p != term))
  {
    return false;
  }
  *out = neg ? -v : v;
  *pp = p + 1;
  return true;
}

static bool cmd_fresh(void)
{
  return cmd_valid && ((HAL_GetTick() - cmd_tick) <= MOTOR_WATCHDOG_MS);
}

/* --------------------------------------------------------------- control */

/* Level the drive rules use: in slow mode a sensor error is not level 3,
 * the speed cap from motor_set_limit() protects instead. */
static uint8_t drive_level(void)
{
  return slow_active ? 0U : obst_level;
}

/* Enter/leave slow mode as the sensor fails/recovers; true on a change */
static bool update_slow(void)
{
  bool want = fail_slow && !tof_ok();

  if (want == slow_active)
  {
    return false;
  }
  slow_active = want;
  motor_set_limit(want ? LINK_TOF_FAIL_MAX_PCT : 100);
  tof_set_retry_ms(want ? LINK_TOF_RETRY_SLOW_MS : TOF_RETRY_MS);
  if (want)
  {
    printf("\r\nLINK: ToF failed, driving slow (max %d%%)\r\n", LINK_TOF_FAIL_MAX_PCT);
  }
  else
  {
    printf("\r\nLINK: slow mode off\r\n");
  }
  return true;
}

static void stop_for_level3(void)
{
  int l, r;

  motor_get_target(&l, &r);
  if ((l != 0) || (r != 0))
  {
    motor_estop();
  }
  out_left = 0;
  out_right = 0;
}

static void apply_drive(void)
{
  int l = cmd_left;
  int r = cmd_right;
  uint8_t lvl = drive_level();

  if (lvl >= 3U)
  {
    stop_for_level3();
    return;
  }
  if (cmd_mode == MODE_STOP)
  {
    l = 0;
    r = 0;
  }
  else if (cmd_mode != MODE_DOCKING)
  {
    if ((lvl == 2U) && ((l + r) > 0))
    {
      l = 0;
      r = 0;
    }
    else if (lvl == 1U)
    {
      if (l > LINK_LEVEL1_MAX_PCT) l = LINK_LEVEL1_MAX_PCT;
      if (r > LINK_LEVEL1_MAX_PCT) r = LINK_LEVEL1_MAX_PCT;
    }
  }
  out_left = l;
  out_right = r;
  motor_set(l, r);   /* also feeds the watchdog, including mode 0 */
}

static void handle_frame(char *line, uint16_t len)
{
  char *star;
  const char *p;
  int hi, lo, l, r, m;

  /* line = "$<body>*XX" */
  if ((len < 4U) || (line[0] != '$') || (line[len - 3U] != '*'))
  {
    rx_bad++;
    return;
  }
  star = &line[len - 3U];
  hi = hex_upper(star[1]);
  lo = hex_upper(star[2]);
  if ((hi < 0) || (lo < 0) || (memchr(line + 1, '*', len - 4U) != NULL) ||
      (xor_sum(line + 1, (size_t)(star - (line + 1))) != (uint8_t)((hi << 4) | lo)))
  {
    rx_bad++;
    return;
  }
  *star = '\0';

  if ((line[1] != 'M') || (line[2] != ','))
  {
    return;   /* valid frame of another type: ignore */
  }
  p = &line[3];
  if (!parse_field(&p, ',', &l) || !parse_field(&p, ',', &r) || !parse_field(&p, '\0', &m) ||
      (l < -100) || (l > 100) || (r < -100) || (r > 100) || (m < 0) || (m > 2))
  {
    rx_bad++;
    return;
  }

  rx_ok++;
  cmd_valid = true;
  cmd_left = l;
  cmd_right = r;
  cmd_mode = m;
  cmd_tick = HAL_GetTick();
  strcpy(cmd_text, line);

  console_link_takeover();     /* upper board wins over the test sequence */
  if (!console_manual_active())
  {
    apply_drive();
  }
}

static void rx_process(void)
{
  char c;

  while (rx_get(&c))
  {
    if (c == '$')
    {
      /* start (or resync) a frame */
      line_buf[0] = c;
      line_len = 1U;
      line_active = true;
      line_overflow = false;
    }
    else if ((c == '\r') || (c == '\n'))
    {
      if (line_active && !line_overflow)
      {
        line_buf[line_len] = '\0';
        handle_frame(line_buf, line_len);
      }
      else if (line_overflow)
      {
        rx_bad++;
      }
      line_active = false;
    }
    else if (line_active && !line_overflow)
    {
      if (line_len < LINK_LINE_MAX)
      {
        line_buf[line_len++] = c;
      }
      else
      {
        line_overflow = true;
      }
    }
  }
}

/* -------------------------------------------------------------- obstacle */

static uint8_t level_of(uint32_t mm)
{
  if (mm < LINK_LEVEL3_MM)  return 3U;
  if (mm < LINK_LEVEL2_MM)  return 2U;
  if (mm <= LINK_LEVEL1_MM) return 1U;
  return 0U;
}

/* Rise immediately, drop only when LINK_HYST_MM further than the threshold
 * for LINK_LEVEL_DROP_MS (stops level flicker and repeated voice alerts).
 * A sensor error counts as level 3. */
static uint8_t level_update(uint32_t mm, uint8_t cur)
{
  static bool     drop_pending;
  static uint32_t drop_since;
  static bool     was_error;
  uint8_t raw = tof_ok() ? level_of(mm) : 3U;
  uint8_t hold;
  uint32_t now = HAL_GetTick();

  if (was_error && tof_ok())
  {
    /* level 3 came from the sensor error, not a distance: drop at once */
    was_error = false;
    drop_pending = false;
    return raw;
  }
  was_error = !tof_ok();
  if (raw >= cur)
  {
    drop_pending = false;
    return raw;
  }
  hold = level_of((mm > LINK_HYST_MM) ? (mm - LINK_HYST_MM) : 0U);
  if (hold >= cur)
  {
    drop_pending = false;
    return cur;
  }
  if (!drop_pending)
  {
    drop_pending = true;
    drop_since = now;
  }
  if ((now - drop_since) < LINK_LEVEL_DROP_MS)
  {
    return cur;
  }
  drop_pending = false;
  return hold;
}

/* -------------------------------------------------------------------- TX */

static bool send_status(void)
{
  char body[16];
  unsigned dist = (obst_dist >= TOF_NO_OBJECT_MM) ? 0U : obst_dist;
  int n;

  if (link_huart->gState != HAL_UART_STATE_READY)
  {
    return false;   /* previous frame still sending, retry next poll */
  }
  snprintf(body, sizeof(body), "O,%u,%u", (unsigned)obst_level, dist);
  n = snprintf(tx_buf, sizeof(tx_buf), "$%s*%02X\r\n", body,
               (unsigned)xor_sum(body, strlen(body)));
  return HAL_UART_Transmit_IT(link_huart, (uint8_t *)tx_buf, (uint16_t)n) == HAL_OK;
}

/* ------------------------------------------------------------------ API */

void link_init(UART_HandleTypeDef *huart)
{
  link_huart = huart;
  rx_head = rx_tail = 0U;
  line_active = false;
  cmd_valid = false;
  cmd_mode = MODE_STOP;
  cmd_text[0] = '\0';

  tof_init();
  obst_dist = tof_read_mm();
  obst_level = level_update(obst_dist, 0U);
  tof_tick = HAL_GetTick();
  tx_pending = true;

  /* Stop the wheels if no valid $M for MOTOR_WATCHDOG_MS */
  motor_watchdog_enable(true);
  rx_arm();
}

void link_poll(void)
{
  uint32_t now;
  bool level_changed = false;

  rx_process();
  tof_poll();
  now = HAL_GetTick();

  if ((now - tof_tick) >= LINK_TOF_PERIOD_MS)
  {
    uint8_t lvl;

    tof_tick = now;
    obst_dist = tof_read_mm();
    lvl = level_update(obst_dist, obst_level);
    if (lvl != obst_level)
    {
      obst_level = lvl;
      level_changed = true;
      tx_pending = true;
    }
  }
  if (update_slow())
  {
    level_changed = true;
  }

  if (drive_level() >= 3U)
  {
    stop_for_level3();   /* also blocks console commands */
  }
  else if (console_manual_active())
  {
    motor_feed();        /* keep manual console commands alive */
  }
  else if (level_changed && cmd_fresh())
  {
    apply_drive();       /* re-apply the last $M with the new limits */
  }

  /* a broken sensor stops the wheels but is not a person to talk to */
  alert_set(tof_ok() ? obst_level : 0U, (uint8_t)cmd_mode);

  if ((now - tx_tick) >= LINK_TX_PERIOD_MS)
  {
    tx_pending = true;
  }
  if (tx_pending && send_status())
  {
    tx_pending = false;
    tx_tick = now;
  }

  if (debug_on && ((now - debug_tick) >= DEBUG_PERIOD_MS))
  {
    debug_tick = now;
    if (cmd_valid)
    {
      printf("LINK rx=%s age=%lums -> L=%+d R=%+d", cmd_text,
             (unsigned long)(now - cmd_tick), out_left, out_right);
    }
    else
    {
      printf("LINK rx=none");
    }
    printf(" lvl=%u dist=%u raw=%u%s%s ok=%lu bad=%lu%s\r\n", (unsigned)obst_level,
           (unsigned)obst_dist, (unsigned)tof_raw_mm(), tof_ok() ? "" : " TOF-ERR",
           slow_active ? " SLOW" : "",
           (unsigned long)rx_ok, (unsigned long)rx_bad,
           console_manual_active() ? " [console]" : "");
  }
}

void link_debug_toggle(void)
{
  debug_on = !debug_on;
  debug_tick = HAL_GetTick() - DEBUG_PERIOD_MS;
}

bool link_debug_enabled(void)
{
  return debug_on;
}

uint8_t link_obstacle_level(void)
{
  return obst_level;
}

void link_set_tof_fail_slow(bool slow)
{
  fail_slow = slow;
  (void)update_slow();
}

bool link_tof_fail_slow(void)
{
  return fail_slow;
}

bool link_slow_active(void)
{
  return slow_active;
}
