/**
  ******************************************************************************
  * @file    console.c
  * @brief   Line-based serial console on USART3 for testing the L298N driver.
  *          RX: 1-byte HAL_UART_Receive_IT -> ring buffer (ISR),
  *          line assembly / echo / command handling in console_poll().
  ******************************************************************************
  */
#include "console.h"
#include "motor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define RX_RING_SIZE   128U   /* power of two */
#define LINE_MAX       48U
#define SPEED_STEP     10
#define SPEED_DEFAULT  50

typedef enum
{
  MODE_STOP = 0,
  MODE_FWD,
  MODE_BACK,
  MODE_SPIN_L,
  MODE_SPIN_R,
  MODE_MANUAL
} drive_mode_t;

typedef struct
{
  const char *name;
  int8_t      left;
  int8_t      right;
  uint16_t    ms;
} test_step_t;

static const test_step_t test_steps[] =
{
  { "Left forward",   60,   0, 2000 },
  { "Stop",            0,   0, 1000 },
  { "Left backward", -60,   0, 2000 },
  { "Stop",            0,   0, 1000 },
  { "Right forward",   0,  60, 2000 },
  { "Stop",            0,   0, 1000 },
  { "Right backward",  0, -60, 2000 },
  { "Stop",            0,   0, 1000 },
  { "Forward 40%",    40,  40, 2000 },
  { "Forward 70%",    70,  70, 2000 },
  { "Forward 100%",  100, 100, 2000 },
  { "Stop",            0,   0, 1000 },
  { "Spin left 60%", -60,  60, 2000 },
  { "Stop",            0,   0, 1000 },
  { "Spin right 60%", 60, -60, 2000 },
  { "Stop",            0,   0, 1000 },
};
#define TEST_STEP_COUNT  (sizeof(test_steps) / sizeof(test_steps[0]))

static UART_HandleTypeDef *con_huart;
static uint8_t rx_byte;
static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head;   /* written by ISR */
static volatile uint16_t rx_tail;   /* written by main loop */

static char     line_buf[LINE_MAX];
static uint16_t line_len;
static char     last_char;

static int          speed = SPEED_DEFAULT;
static drive_mode_t mode = MODE_STOP;

static bool     test_running;
static uint32_t test_index;
static uint32_t test_step_start;

int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;

  if (con_huart != NULL)
  {
    HAL_UART_Transmit(con_huart, &c, 1U, HAL_MAX_DELAY);
  }
  return ch;
}

static void rx_arm(void)
{
  (void)HAL_UART_Receive_IT(con_huart, &rx_byte, 1U);
}

void console_rx_cplt(void)
{
  uint16_t next = (uint16_t)((rx_head + 1U) & (RX_RING_SIZE - 1U));

  if (next != rx_tail)   /* drop byte when full */
  {
    rx_ring[rx_head] = rx_byte;
    rx_head = next;
  }
  rx_arm();
}

void console_rx_error(void)
{
  __HAL_UART_CLEAR_FLAG(con_huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
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

static void print_help(void)
{
  printf("\r\n--- L298N motor test console ---\r\n"
         "  w / s       forward / backward\r\n"
         "  a / d       spin left / spin right (in place)\r\n"
         "  x           stop\r\n"
         "  + / -       speed +/-10%% (applies to current mode)\r\n"
         "  L <n>       left side only, -100..100\r\n"
         "  R <n>       right side only, -100..100\r\n"
         "  t           run automatic test (any input aborts)\r\n"
         "  wd on|off   enable/disable %u ms watchdog\r\n"
         "  db <n>      deadband 0..90 %%\r\n"
         "  inv l|r     toggle left/right invert\r\n"
         "  st          status\r\n"
         "  h / ?       this help\r\n", (unsigned)MOTOR_WATCHDOG_MS);
}

static void print_status(void)
{
  char buf[64];

  motor_status_str(buf, sizeof(buf));
  printf("%s speed=%d%% wd=%s db=%u invL=%u invR=%u\r\n", buf, speed,
         motor_watchdog_is_enabled() ? "on" : "off",
         (unsigned)motor_deadband, (unsigned)MOTOR_L_INVERT, (unsigned)MOTOR_R_INVERT);
}

static void apply_mode(void)
{
  switch (mode)
  {
    case MODE_FWD:    motor_set(speed, speed);   break;
    case MODE_BACK:   motor_set(-speed, -speed); break;
    case MODE_SPIN_L: motor_set(-speed, speed);  break;
    case MODE_SPIN_R: motor_set(speed, -speed);  break;
    case MODE_STOP:   motor_set(0, 0);           break;
    case MODE_MANUAL: default:                   break;
  }
}

static void test_start_step(void)
{
  const test_step_t *s = &test_steps[test_index];

  motor_set(s->left, s->right);
  test_step_start = HAL_GetTick();
  printf("[TEST %lu/%u] %s L=%+d R=%+d\r\n", (unsigned long)(test_index + 1U),
         (unsigned)TEST_STEP_COUNT, s->name, s->left, s->right);
}

static void test_poll(void)
{
  if (!test_running)
  {
    return;
  }
  motor_feed();
  if ((HAL_GetTick() - test_step_start) < test_steps[test_index].ms)
  {
    return;
  }
  test_index++;
  if (test_index < TEST_STEP_COUNT)
  {
    test_start_step();
  }
  else
  {
    test_running = false;
    mode = MODE_STOP;
    motor_set(0, 0);
    printf("[TEST] done\r\n");
    print_status();
  }
}

/* Parse a signed integer argument; returns false if missing or malformed. */
static bool parse_int(const char *s, int *out)
{
  char *end;
  long v;

  while (*s == ' ')
  {
    s++;
  }
  if (*s == '\0')
  {
    return false;
  }
  v = strtol(s, &end, 10);
  while (*end == ' ')
  {
    end++;
  }
  if (*end != '\0')
  {
    return false;
  }
  *out = (int)v;
  return true;
}

static void handle_line(char *line)
{
  char *p = line;
  size_t n;
  int v;
  int l, r;

  /* lower-case and trim */
  for (char *q = line; *q != '\0'; q++)
  {
    *q = (char)tolower((unsigned char)*q);
  }
  while (*p == ' ')
  {
    p++;
  }
  n = strlen(p);
  while ((n > 0U) && (p[n - 1U] == ' '))
  {
    p[--n] = '\0';
  }

  if (test_running)
  {
    test_running = false;
    mode = MODE_STOP;
    motor_set(0, 0);
    printf("[TEST] aborted\r\n");
    print_status();
    return;
  }

  if (n == 0U)
  {
    return;
  }

  if (strcmp(p, "w") == 0)       { mode = MODE_FWD;    apply_mode(); }
  else if (strcmp(p, "s") == 0)  { mode = MODE_BACK;   apply_mode(); }
  else if (strcmp(p, "a") == 0)  { mode = MODE_SPIN_L; apply_mode(); }
  else if (strcmp(p, "d") == 0)  { mode = MODE_SPIN_R; apply_mode(); }
  else if (strcmp(p, "x") == 0)  { mode = MODE_STOP;   apply_mode(); }
  else if ((strcmp(p, "+") == 0) || (strcmp(p, "-") == 0))
  {
    speed += (p[0] == '+') ? SPEED_STEP : -SPEED_STEP;
    if (speed > 100) speed = 100;
    if (speed < 0)   speed = 0;
    apply_mode();
  }
  else if (((p[0] == 'l') || (p[0] == 'r')) && ((p[1] == ' ') || isdigit((unsigned char)p[1]) || (p[1] == '-') || (p[1] == '+')))
  {
    if (!parse_int(p + 1, &v) || (v < -100) || (v > 100))
    {
      printf("ERR: usage L <n> / R <n>, n = -100..100\r\n");
      return;
    }
    motor_get_target(&l, &r);
    if (p[0] == 'l')
    {
      l = v;
    }
    else
    {
      r = v;
    }
    mode = MODE_MANUAL;
    motor_set(l, r);
  }
  else if (strcmp(p, "t") == 0)
  {
    mode = MODE_MANUAL;
    test_running = true;
    test_index = 0U;
    printf("[TEST] start, send any line to abort\r\n");
    test_start_step();
    return;
  }
  else if (strcmp(p, "wd on") == 0)  { motor_watchdog_enable(true); }
  else if (strcmp(p, "wd off") == 0) { motor_watchdog_enable(false); }
  else if ((strncmp(p, "db", 2) == 0) && ((p[2] == ' ') || isdigit((unsigned char)p[2])))
  {
    if (!parse_int(p + 2, &v) || (v < 0) || (v > 90))
    {
      printf("ERR: usage db <n>, n = 0..90\r\n");
      return;
    }
    motor_deadband = (uint8_t)v;
  }
  else if (strcmp(p, "inv l") == 0) { MOTOR_L_INVERT = MOTOR_L_INVERT ? 0U : 1U; }
  else if (strcmp(p, "inv r") == 0) { MOTOR_R_INVERT = MOTOR_R_INVERT ? 0U : 1U; }
  else if (strcmp(p, "st") == 0)    { /* status printed below */ }
  else if ((strcmp(p, "h") == 0) || (strcmp(p, "?") == 0))
  {
    print_help();
  }
  else
  {
    printf("ERR: unknown command '%s' (h = help)\r\n", p);
    return;
  }

  print_status();
}

void console_init(UART_HandleTypeDef *huart)
{
  con_huart = huart;
  rx_head = rx_tail = 0U;
  line_len = 0U;
  last_char = '\0';
  setvbuf(stdout, NULL, _IONBF, 0);

  print_help();
  print_status();
  printf("> ");
  rx_arm();
}

void console_poll(void)
{
  char c;

  while (rx_get(&c))
  {
    if ((c == '\r') || (c == '\n'))
    {
      /* treat CR LF as a single line end */
      if (!((c == '\n') && (last_char == '\r')))
      {
        printf("\r\n");
        line_buf[line_len] = '\0';
        handle_line(line_buf);
        line_len = 0U;
        printf("> ");
      }
    }
    else if ((c == '\b') || (c == 0x7F))
    {
      if (line_len > 0U)
      {
        line_len--;
        printf("\b \b");
      }
    }
    else if (isprint((unsigned char)c) && (line_len < (LINE_MAX - 1U)))
    {
      line_buf[line_len++] = c;
      putchar(c);
    }
    last_char = c;
  }

  test_poll();

  if (motor_watchdog_tripped())
  {
    mode = MODE_STOP;
    test_running = false;
    printf("\r\nWATCHDOG: no motor_set/feed for > %u ms, motors stopped\r\n",
           (unsigned)MOTOR_WATCHDOG_MS);
    print_status();
    printf("> ");
  }
}
