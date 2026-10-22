# Motor_driver_stm32

L298N motor driver test firmware for **NUCLEO-F767ZI** (STM32CubeIDE / CubeMX project),
for a 4WD skid-steer robot: two left motors in parallel on channel A, two right motors in parallel on channel B.

## Hardware / wiring

| L298N | NUCLEO-F767ZI | Function |
|-------|---------------|----------|
| ENA   | PD14 (TIM4_CH3) | Left PWM |
| ENB   | PD15 (TIM4_CH4) | Right PWM |
| IN1   | PF13 | Left direction A |
| IN2   | PF12 | Left direction B |
| IN3   | PF14 | Right direction A |
| IN4   | PF15 | Right direction B |
| GND   | GND  | **Common ground (required)** |

- Remove the ENA/ENB jumpers on the L298N so the PWM pins actually control speed.
- Power the motors from a separate battery on the L298N 12V input. Do not power motors from the Nucleo.
- The console runs on USART3 (PD8/PD9), which is the ST-LINK Virtual COM Port on the USB cable. No extra wiring is needed.

## Configuration (Motor_driver.ioc)

- SYSCLK 216 MHz (HSI + PLL), APB1 /4 (timer clock 108 MHz), APB2 /2
- TIM4: PSC 107, ARR 999, PWM 1 kHz, PWM mode 1, ARR preload enabled
- TIM6: PSC 10799, ARR 199, 20 ms interrupt for `motor_tick()`
- USART3: 115200 8N1, interrupt enabled

## Source layout

| File | Description |
|------|-------------|
| `Core/Inc/motor.h`, `Core/Src/motor.c` | L298N driver: ramping, deadband, invert, watchdog, e-stop |
| `Core/Inc/console.h`, `Core/Src/console.c` | Serial console on USART3, `printf` redirect, automatic test sequence |
| `Core/Src/main.c` | Calls init/poll and HAL callbacks, edited in `USER CODE` sections only |

### Motor API

```c
motor_init(&htim4, TIM_CHANNEL_3, TIM_CHANNEL_4);
motor_set(left, right);          // -100..100 %, negative = reverse; also feeds the watchdog
motor_tick();                    // from the TIM6 ISR every 20 ms
motor_estop();                   // immediate stop, no ramp (safe from an EXTI ISR)
motor_feed();                    // feed the watchdog without changing speed
motor_watchdog_enable(true);     // stop if no set/feed for 500 ms (default: off)
motor_watchdog_tripped();        // returns true once after a watchdog stop
motor_status_str(buf, len);      // "L=+60% R=+60% (IN1=1 IN2=0 IN3=1 IN4=0)"
```

Behavior:
- **Ramp:** speed changes by at most 5% per tick (20 ms). A direction change always stops at 0 first.
- **Direction:** forward IN=1/0, reverse IN=0/1, stop IN=0/0 (coast) with CCR=0.
- **PWM:** CCR = % × (ARR+1) / 100, so 100% gives 1000 and the output stays high.
- **Globals:**
  - `MOTOR_L_INVERT` / `MOTOR_R_INVERT` reverse a whole side.
  - `motor_deadband` (%) maps 1..100% to deadband..100%.

## Build & flash

1. Open STM32CubeIDE and choose **File → Import → Existing Projects into Workspace**, then select this folder.
2. Run **Project → Build Project**.
3. Connect the Nucleo over USB and choose **Run → Debug As → STM32 C/C++ Application**, or use Run.

> If you regenerate code from the `.ioc`, keep user code inside the `/* USER CODE BEGIN */ ... END` sections only.

## Testing with Tera Term

1. Open the **STMicroelectronics STLink Virtual COM Port** at **115200, 8N1**.
2. In Setup → Terminal, set New-line **Receive = AUTO** (or CR).
3. Press the black RESET button on the board. The help text and a status line appear:
   ```
   L=+0% R=+0% (IN1=0 IN2=0 IN3=0 IN4=0) speed=50% wd=off db=0 invL=0 invR=0
   ```

Commands are case-insensitive. Press Enter after each one:

| Command | Action |
|---------|--------|
| `w` / `s` | Forward / backward |
| `a` / `d` | Spin left / spin right in place |
| `x` | Stop |
| `+` / `-` | Speed ±10% (default 50%), applied to the current mode immediately |
| `L <n>` / `R <n>` | Drive one side only (-100..100), keeping the other side's value |
| `t` | Run the automatic 16-step test. Any input aborts it and stops the motors |
| `wd on` / `wd off` | Enable/disable the 500 ms watchdog |
| `db <n>` | Set deadband 0..90% |
| `inv l` / `inv r` | Toggle left/right invert |
| `st` | Show status |
| `h` / `?` | Help |

### Recommended first test (lift the wheels off the ground first)

1. Run `L 30` and confirm both left wheels spin forward. If they spin backward, run `inv l`. Then run `x`.
2. Run `R 30` the same way, and use `inv r` if needed.
3. Run `w`, `s`, `a`, `d`, then use `+` / `-` to check speed changes.
4. If the wheels don't move at low speed, set a deadband, e.g. `db 20`.
5. Run `t` to execute the automatic sequence (about 26 s). It prints `[TEST n/16] ...` for each step.
6. Run `wd on`, then `w`. The wheels should stop by themselves after about 0.5 s and print `WATCHDOG: ...`. Run `wd off` to return to manual test mode.

> The `inv` and `db` values live in RAM and are lost on reset. Once you know the right values, set them as defaults in `Core/Src/motor.c`.

## Roadmap

- USART2: receive `$D,<v mm/s>,<w deg/s>*cs` from the upper board, with the watchdog enabled
- ToF sensor on EXTI calling `motor_estop()`
