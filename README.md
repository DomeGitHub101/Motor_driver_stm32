# Motor_driver_stm32

Lower-board firmware **v1.0.0** for **NUCLEO-F767ZI** (STM32CubeIDE / CubeMX project): L298N motor driver, front ToF, voice alert, and UART link to the upper board,
for a 4WD skid-steer robot: the two right motors are in parallel on channel A (OUT1/OUT2) and the two left motors are in parallel on channel B (OUT3/OUT4).

## Hardware / wiring

| L298N | NUCLEO-F767ZI | Function |
|-------|---------------|----------|
| ENA   | PD14 (TIM4_CH3) | Right PWM |
| ENB   | PD15 (TIM4_CH4) | Left PWM |
| IN1   | PF13 | Right direction A |
| IN2   | PF12 | Right direction B |
| IN3   | PF14 | Left direction A |
| IN4   | PF15 | Left direction B |
| GND   | GND  | **Common ground (required)** |

- If the motors are rewired so that channel A = left, set `MOTOR_SWAP_SIDES` to `0` in `Core/Src/motor.c`. All other code (console, `$M`, `MOTOR_L_INVERT` / `MOTOR_R_INVERT`) always uses the robot's real left and right.

- Remove the ENA/ENB jumpers on the L298N so the PWM pins actually control speed.
- Power the motors from a separate battery on the L298N 12V input. Do not power motors from the Nucleo.
- The console runs on USART3 (PD8/PD9), which is the ST-LINK Virtual COM Port on the USB cable. No extra wiring is needed.

### Front ToF (VL53L0X module)

| VL53L0X | NUCLEO-F767ZI |
|---------|---------------|
| VIN | 3V3 |
| GND | GND |
| SCL | PB8 (D15) |
| SDA | PB9 (D14) |
| XSHUT | PA0 |
| GPIO1 | not connected |

### Speaker (I2S amplifier, e.g. MAX98357A)

| Amplifier | NUCLEO-F767ZI |
|-----------|---------------|
| BCLK | PD3 (I2S2_CK) |
| LRC / WS | PB12 (I2S2_WS) |
| DIN | PB15 (I2S2_SD) |
| VIN | 5V |
| GND | GND |

- Connect the speaker (4–8 Ω) to the amplifier's + / − outputs only. Never connect either output to GND (BTL output).
- If the board resets while a voice plays (weak USB / power bank), lower `ALERT_VOLUME_PERCENT` in `Core/Inc/alert.h` or give the amplifier its own 5 V supply.

## Configuration (Motor_driver.ioc)

- SYSCLK 216 MHz (HSI + PLL), APB1 /4 (timer clock 108 MHz), APB2 /2
- TIM4: PSC 107, ARR 999, PWM 1 kHz, PWM mode 1, ARR preload enabled
- TIM6: PSC 10799, ARR 199, 20 ms interrupt for `motor_tick()`
- USART3: 115200 8N1, interrupt enabled (console)
- USART2: PD5/PD6, 115200 8N1, interrupt enabled (upper board)
- I2C1: PB8/PB9, 100 kHz (ToF)
- I2S2: master TX, Philips, 16-bit, 16 kHz, PD3/PB12/PB15, DMA1 Stream4 circular (speaker), PLLI2S 96 MHz
- PA0: GPIO output `XSHUT`, initial Low

## Source layout

| File | Description |
|------|-------------|
| `Core/Inc/motor.h`, `Core/Src/motor.c` | L298N driver: ramping, deadband, invert, watchdog, e-stop |
| `Core/Inc/console.h`, `Core/Src/console.c` | Serial console on USART3, `printf` redirect, automatic test sequence |
| `Core/Inc/link.h`, `Core/Src/link.c` | USART2 link to the upper board: `$M` commands in, `$O` obstacle status out, drive priority |
| `Core/Inc/tof.h`, `Core/Src/tof.c` | Front ToF: one VL53L0X in continuous mode, non-blocking `tof_poll()`, `tof_read_mm()`, `tof_ok()` |
| `Core/Inc/vl53l0x_simple.h`, `Core/Src/vl53l0x_simple.c` | Minimal VL53L0X register driver (init sequence based on Pololu's library, MIT) |
| `Core/Inc/alert.h`, `Core/Src/alert.c` | `alert_set(level, mode)`: plays voice clips on the I2S speaker |
| `Core/Inc/voice500.h`, `voice1000.h`, `voice2000.h` | 16 kHz 16-bit mono PCM clips (included only by `alert.c`) |
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
  - `motor_deadband` (%) maps 1..100% to deadband..100%. Default 45: measured start was left 42%, right 34%.

## Build & flash

1. Open STM32CubeIDE and choose **File → Import → Existing Projects into Workspace**, then select this folder.
2. Run **Project → Build Project**.
3. Connect the Nucleo over USB and choose **Run → Debug As → STM32 C/C++ Application**, or use Run.

> If you regenerate code from the `.ioc`, keep user code inside the `/* USER CODE BEGIN */ ... END` sections only.

## Testing with Tera Term

1. Open the **STMicroelectronics STLink Virtual COM Port** at **115200, 8N1**.
2. In Setup → Terminal, set New-line **Receive = AUTO** (or CR).
3. Press the black RESET button on the board. You should see the banner (`--- Lower board FW 1.0.0 ... ---`), the help text, `TOF: VL53L0X ready`, and a status line:
   ```
   L=+0% R=+0% (IN1=0 IN2=0 IN3=0 IN4=0) speed=50% wd=on db=45 invL=0 invR=0 lvl=0
   ```

**Boot:** the wheels stay stopped (PWM = 0, IN1..IN4 = 0) until a command arrives. The automatic test never starts by itself: type `t` + Enter in the console.
A valid `$M` from the upper board aborts the test immediately and hands control to the upper board.

Commands are case-insensitive. Press Enter after each one:

| Command | Action |
|---------|--------|
| `w` / `s` | Forward / backward |
| `a` / `d` | Spin left / spin right in place |
| `x` | Stop |
| `+` / `-` | Speed ±10% (default 50%), applied to the current mode immediately |
| `L <n>` / `R <n>` | Drive one side only (-100..100), keeping the other side's value |
| `t` | Run the automatic 16-step test. Any console input aborts it and stops the motors; a valid `$M` aborts it and hands control to the upper board |
| `wd on` / `wd off` | Enable/disable the 500 ms watchdog |
| `db <n>` | Set deadband 0..90% |
| `inv l` / `inv r` | Toggle left/right invert |
| `link` | Toggle printing of received `$M`, actual wheel output and obstacle level (max 2 lines/s) |
| `tof <mm>` / `tof off` | Simulate a ToF distance / go back to the real sensor |
| `i2c` | Scan I2C1 (PB8/PB9). The ToF should answer at `0x29`. `none` means a wiring or power problem |
| `st` | Show status |
| `h` / `?` | Help |

### Recommended first test (lift the wheels off the ground first)

1. Run `L 30` and confirm both left wheels spin forward. If they spin backward, run `inv l`. Then run `x`.
2. Run `R 30` the same way, and use `inv r` if needed.
3. Run `w`, `s`, `a`, `d`, then use `+` / `-` to check speed changes.
4. The deadband defaults to 45%. If a wheel still does not move at low speed, raise it (e.g. `db 50`). If it jumps too hard, lower it.
5. Run `t` to execute the automatic sequence (about 26 s). It prints `[TEST n/16] ...` for each step.
6. The watchdog is **on** at boot because the link enables it. A console motion command (`w` `s` `a` `d` `x` `+` `-` `L` `R`) overrides the upper board for 2 s after you type it, then hands control back. With no upper board connected, the wheels stop about 0.5 s after that (`WATCHDOG: ...`), so a manual command runs for about 2.5 s. Run `wd off` to make manual commands run until `x`.

> The `inv` and `db` values live in RAM and are lost on reset. Once you know the right values, set them as defaults in `Core/Src/motor.c`.

## Upper board link (USART2)

### Wiring

| Lower board (this Nucleo) | Upper board |
|---------------------------|-------------|
| PD5 (USART2_TX) | RX |
| PD6 (USART2_RX) | TX |
| GND | GND (**common ground required**) |

TX and RX cross over. Both boards use 3.3 V logic. Settings: 115200 8N1.

### Protocol

Every message is one ASCII line: `$<body>*XX\r\n`

- `XX` = XOR of every character between `$` and `*` (both excluded), as 2 upper-case hex digits.
- A line is dropped entirely if the checksum is wrong, the format is wrong, or it is longer than 47 characters.

| Direction | Message | Meaning |
|-----------|---------|---------|
| Upper → lower, every ~100 ms and on change | `$M,left,right,mode*XX` | Wheel % -100..100 (+ = forward). mode 0 = stop/wait, 1 = run, 2 = docking at the table |
| Lower → upper, every 200 ms and on level change | `$O,level,dist_mm*XX` | Obstacle level 0..3 and distance (0 = no object) |

Examples: `$M,0,0,0*51`, `$M,40,32,1*55`, `$M,-30,30,1*7D`, `$O,0,0*4F`, `$O,2,450*4C`

The upper board treats the lower board as lost if no `$O` arrives for over 1 s, and then commands 0.

### Obstacle level (ToF)

| Distance | Level |
|----------|-------|
| > 1000 mm | 0 |
| 600–1000 mm | 1 |
| 300–600 mm | 2 |
| < 300 mm | 3 |

- The level rises immediately.
- It drops only after the distance is 50 mm past the threshold (hysteresis).
- Tune the thresholds with the `LINK_LEVEL*_MM` / `LINK_HYST_MM` macros in `Core/Inc/link.h`.
- Readings are clamped to 2000 mm (no object).
- **Sensor error:** if init fails, or no new sample arrives for 300 ms, the level is forced to 3 (wheels stop) and `$O,3,0` is sent. The console prints `TOF: ...`, and the sensor is re-initialized every 2 s until it works. The speaker stays silent for a sensor error.

### Drive priority

1. **Level 3**: `motor_estop()`. Ignores the upper board, the console and mode 2.
2. **Level 2** (mode ≠ 2): no forward. If `left + right > 0`, both wheels go to 0. Reverse and spinning in place are still allowed.
3. **Level 1** (mode ≠ 2): forward limited to 30% per wheel.
4. **mode 0**: `motor_set(0, 0)`.
5. **Otherwise**: `motor_set(left, right)`.

Other rules:
- **Watchdog:** on from boot. With no valid `$M` for 500 ms, the wheels ramp to 0. `motor_set()` feeds it on every valid `$M`, including mode 0.
- **Console override:** a console motion command (`w` `s` `a` `d` `x` `+` `-` `L` `R`) wins over `$M` until 2 s after the last command typed. Level 3 still applies. The `t` test is the exception: a valid `$M` aborts it immediately, so with the upper board connected `t` stops right away.

### Voice alert (speaker)

| Level | Clip | Says | When |
|-------|------|------|------|
| 1 | `voice2000` | "ขอโทษครับ กรุณาหลบหน่อยครับ" | Once when the level rises to 1 |
| 2 | `voice1000` | "กรุณาหลบหน่อยครับ" | Once when the level rises to 2 |
| 3 | `voice500` | "หลบไปดิ้" | Every 3 s while at level 3 |

- A voice plays only when the obstacle gets **closer** (the level rises). Moving away is silent.
- Mode 2 (docking): silent at every level, because the ToF sees the table.
- Each alert is also printed on the console. To see the Thai text, set Tera Term to UTF-8 (Setup → General / Terminal → Coding).
- Volume: `ALERT_VOLUME_PERCENT` in `Core/Inc/alert.h` (default 50).

### Testing the link with a USB-UART adapter instead of the upper board

**Always lift the wheels off the ground.**

1. Wire the USB-UART to the Nucleo: USB-UART TX → PD6, RX → PD5, GND → GND. Use a 3.3 V adapter.
2. Open two Tera Term windows, both at 115200 8N1:
   - Window A: ST-LINK VCP (console). Type `link` to see what is received.
   - Window B: the USB-UART. Set New-line Transmit = **CR+LF**.
3. You should see `$O,0,0*4F` arriving in window B every 200 ms.
4. Paste these lines into window B (Edit → Paste, or a Tera Term macro that sends one every 100 ms):

| Send | Expected |
|------|----------|
| `$M,30,30,1*50` | Both sides forward at 30% |
| (stop sending) | Wheels stop within about 0.5 s, and window A shows `WATCHDOG: ...` |
| `$M,30,30,1*51` (wrong checksum) | No effect, `bad=` count increases |
| `$M,-30,-30,1*50` | Both sides backward |
| `$M,-40,40,1*7D` | Spin left in place |
| `$M,30,30,0*51` | mode 0, wheels stay stopped |
| `$M,30,30,2*53` | Docking mode, forward 30% |

A single pasted line drives the wheels for only 0.5 s because of the watchdog. For longer runs, send the line repeatedly with a Tera Term macro, e.g.:
```
:loop
sendln '$M,30,30,1*50'
mpause 100
goto loop
```

5. Test the obstacle levels by simulating distances in window A while `$M,60,60,1*50` is being sent repeatedly:

| Console | Expected |
|---------|----------|
| `tof 800` | Level 1, forward drops from 60% to 30%, `$O,1,800*..`, speaker: "ขอโทษครับ กรุณาหลบหน่อยครับ" |
| `tof 500` | Level 2, wheels stop, speaker: "กรุณาหลบหน่อยครับ" once |
| `tof 200` | Level 3, e-stop, speaker: "หลบไปดิ้" every 3 s |
| `tof off` | Back to the real sensor. Put a hand in front of it to check the real levels |

## Troubleshooting

| Symptom | Likely cause |
|---------|--------------|
| `TOF: init failed (err 10)` and `i2c` prints `none` | The sensor does not answer: loose or wrong SCL/SDA wire (SCL = D15/PB8, SDA = D14/PB9), no 3.3 V at the module, or XSHUT not on PA0. The wheels stay blocked (level 3) until it works. Type `tof 2000` to test the motors without the sensor. |
| Stuck at level 3 with `dist` around 150 mm | Something is in front of the sensor: the protective film still on, a cable, the chassis, or the sensor tilted towards the floor. |
| `bad=` rises while `rx=none` | Noise on USART2 RX (PD6 floating, upper board not connected), or wrong wiring/baud/GND to the upper board. |
| Board resets while a voice plays | 5 V supply too weak for the amplifier. Lower `ALERT_VOLUME_PERCENT` or power the amplifier separately. |
| Wheels run for only ~2.5 s from console commands | Expected: the watchdog is on. Use `wd off` for manual tests. |

## Roadmap

- ToF GPIO1 on EXTI calling `motor_estop()` for a faster stop
