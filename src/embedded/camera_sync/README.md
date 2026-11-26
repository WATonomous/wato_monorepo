# camera_sync: camera trigger board firmware

Fires a PPS-locked 20 Hz trigger pulse into all cameras at the same instant, and reports every
pulse to the computer (issues #561, #564). One source tree builds for both the **NUCLEO-F767ZI**
and the **NUCLEO-H753ZI**.

```text
PwrPak7 PPS --> TIM2 capture --+                       +--> CAM_TRIG_1..12 (one BSRR write)
                               |  STM32: pulse grid    |
camera strobes --> EXTI -------+  locked to PPS        +--> report lines: UDP 10.8.0.1:5800
                                                            and ST-LINK USB serial
```

The computer side is `camera_trigger_node` (PR #611). The lines it reads are specified in
[docs/report_protocol.md](docs/report_protocol.md).

## What it does

- **20 pulses per second, locked to PPS.** TIM2 runs freely at the timer clock (108 MHz on the
  F767, 200 MHz on the H753). The PPS edge is captured in hardware. Pulse `i` of each second fires
  at `PPS + i * 50 ms`, with pulse 0 at the predicted PPS edge. In host simulation (20 ns PPS
  jitter, a +37 ppm crystal, counter wrap-around) every locked pulse is within 0.2 us of the ideal
  grid. On hardware, add a fixed interrupt delay of roughly 0.1-0.2 us, the same for every pulse.
- **All outputs switch together.** Every trigger output is on one GPIO port and set by one BSRR
  write, so there is no skew between cameras.
- **Holdover.** If PPS is lost, pulses continue at the last measured rate and are marked
  `pps_locked = 0`. When PPS comes back on the same phase they relock without a jump. Without PPS
  at all, the board free-runs at the nominal rate.
- **Strobe readback.** Each camera's strobe (exposure) output is latched per pulse into
  `strobe_mask`, so a camera that missed a trigger shows up immediately.
- **Reporting never delays a pulse.** The timer interrupt has the highest priority and only queues
  records. Formatting and sending (UDP and USB serial) run in the main loop. Ethernet is polled, so
  it has no interrupt that could get in the way.

## Layout

| Path | What | Chip-specific? |
|---|---|---|
| `config/cs_pins.h` | **Pin map**: trigger outputs, strobe inputs, PPS input | no, one map for both boards |
| `config/cs_config.h` | Rate, pulse width, PPS lock tuning, network addresses | no |
| `app/` | Portable C: PPS scheduler (`cs_sched`), report lines (`cs_report`), minimal UDP/ARP/ping (`cs_net`), pin checks | no, unit-tested on the host |
| `port/stm32/` | HAL-based port: timer and interrupts, GPIO, UART, Ethernet, main loop | shared by F7 and H7 |
| `boards/<board>/` | Clocks, board pins (LEDs, Ethernet, UART), HAL config, startup, linker script | yes |
| `Drivers/` | ST HAL and CMSIS (vendored subsets) | yes |
| `tests/` | Host unit tests | no |

The F7 and H7 HALs have the same API for everything used here (GPIO, UART, ETH, RCC), so the
port is one set of files with a handful of `CS_FAMILY_H7` lines. Each `boards/` directory holds
what really differs: clock tree, memory map, HAL configuration.

## Pin map

Edit only [config/cs_pins.h](config/cs_pins.h). The defaults are the same on both boards:

| Role | Pins | Notes |
|---|---|---|
| CAM_TRIG_1..12 | PE2..PE13 | One port, switched together. 3.3 V push-pull outputs to the driver board, never directly to a camera |
| STB1..12 (strobe in) | PF0..PF11 | Bit k-1 of `strobe_mask`. Pull-down, rising edge |
| PPS in | PA0 (TIM2_CH1) | Rising edge |
| Ethernet (fixed by the board) | PA1, PA2, PA7, PB13, PC1, PC4, PC5, PG11, PG13 | RMII to the LAN8742A |
| ST-LINK serial (fixed) | PD8, PD9 | USART3 |
| LEDs (fixed) | LD1 PB0, LD2 PB7 (F767) / PE1 (H753), LD3 PB14 | See [LEDs](#leds) |

The build checks the map and fails with a message naming the problem and the port if:

- two roles use the same pin, or a pin the board reserves (Ethernet, ST-LINK UART, SWD, oscillator, USB, LEDs);
- two strobe inputs share a pin number (each pin number has one EXTI line, so PF3 and PG3 can't both be strobes);
- the PPS pin has no TIM2 channel (allowed: PA0, PA5, PA15, PB3, PB10, PA3, PB11);
- no trigger output is defined.

Unused roles are set to `CS_NO_PIN`. Strobes keep their bit positions, so leave unwired inputs out
of the node's `expected_strobe_bits`. **Don't use PC0-PC11 for strobes**: PC1, PC4 and PC5 are
Ethernet pins on both NUCLEO boards.

## Build

Needs CMake 3.20+ and the GNU Arm toolchain: `sudo apt install gcc-arm-none-eabi`, or the one in
STM32CubeCLT / CubeIDE (`export ARM_TOOLCHAIN_DIR=<dir containing bin/arm-none-eabi-gcc>`).

```bash
cd src/embedded/camera_sync
cmake --preset h753 && cmake --build --preset h753   # or f767
```

Output in `build/<preset>/`: `camera_sync_nucleo_<board>.elf`, `.bin` and `.hex`. The firmware
uses about 20 KB of flash.

## Flash

Any of these works on both boards, over the NUCLEO's ST-LINK USB:

- Copy the `.bin` to the NUCLEO's USB drive (drag and drop).
- STM32CubeProgrammer CLI:

  ```bash
  STM32_Programmer_CLI -c port=SWD -w build/h753/camera_sync_nucleo_h753zi.elf -v -rst
  ```

- `st-flash` (stlink 1.7+ for the H753):

  ```bash
  st-flash --reset write build/h753/camera_sync_nucleo_h753zi.bin 0x08000000
  ```

To debug, load the `.elf` in STM32CubeIDE (*File > Import > C/C++ executable*) or VS Code with
Cortex-Debug.

## Check it's working

```bash
stty -F /dev/ttyACM0 115200 raw && cat /dev/ttyACM0   # BOOT once, then 20 TRG lines/s and 1 STAT/s
```

```bash
ping 10.8.0.60
```

```bash
nc -ul 5800   # on the computer (10.8.0.1): the same lines over UDP
```

Then run the computer side: `ros2 launch camera_trigger camera_trigger.launch.yaml`
(`udp_bind_address: 10.8.0.1`, `udp_source_ip: 10.8.0.60`).

## LEDs

| LED | On | Blinking | Off |
|---|---|---|---|
| LD1 green (lock) | locked to PPS | 1 Hz: holdover (PPS lost). Short flash every 2 s: free-running, no PPS yet | |
| LD2 blue/yellow (link) | computer reachable (ARP answered) | 2 Hz: Ethernet link, computer not answering | no Ethernet link |
| LD3 red (fault) | for 1 s after a dropped report line, a skipped pulse or a PPS glitch; steady before a reset after a crash | | normal |

After a crash the board drives the trigger outputs idle, lights LD3 and resets a few tens of
milliseconds later (`CS_RESET_ON_FAULT`), so triggering resumes. The new `BOOT` line tells the node.

## Tests

```bash
cmake --preset host && cmake --build --preset host && ctest --preset host
```

| Test | Covers |
|---|---|
| `sched` | Simulated timer and PPS: lock time, accuracy against the ideal grid, 108 and 200 MHz, HSI error (0.8 %), PPS loss, holdover and rejoin, phase jump and realign, glitch edges, counter wrap-around, skipped pulses |
| `report` | Lines byte for byte against the examples in the node's protocol doc, field limits, STAT, the record queue |
| `net` | ARP request, reply and learning, UDP frame and checksums, ping replies, ignored traffic, MAC from the chip ID |
| `pins_*` | The default map compiles for both boards; each kind of bad map fails with its message |

## Configuration

In [config/cs_config.h](config/cs_config.h):

| Setting | Default | |
|---|---|---|
| `CS_PULSES_PER_SECOND` | 20 | |
| `CS_TRIG_PULSE_US` | 1000 | trigger pulse width |
| `CS_STROBE_WINDOW_US` | 5000 | strobes are counted for a pulse until this long after it, then its `TRG` line is sent |
| `CS_PPS_LOCK_COUNT` | 2 | valid 1 s intervals needed to lock (lock about 2 s after PPS appears) |
| `CS_PPS_LOCK_TOL_PPM` | 100 | a locked edge more than 100 us off the prediction is ignored as a glitch |
| `CS_PPS_TIMEOUT_MS` | 1500 | locked to holdover after this long without a valid edge |
| `CS_NET_IP` / `CS_NET_DEST_IP` | 10.8.0.60 / 10.8.0.1 | static; port 5800 |
| `CS_UART_ENABLE` / `CS_NET_ENABLE` | 1 / 1 | report transports |

Rates, widths and windows are checked at compile time (for example, the window must be shorter
than half the pulse period).

## Before wiring it to the car

Not yet verified on hardware. The build, the host tests and the cross-check against the node's
parser pass, but no board has run this firmware.

- **PPS level and edge.** Check the PwrPak7 PPS voltage against the STM32 input (3.3 V logic,
  5 V tolerant only on FT pins). The edge must match the receiver's polarity: saved NEGATIVE until
  #560 sets `PPSCONTROL ENABLE POSITIVE` and `SAVECONFIG`. Change `CS_PPS_RISING_EDGE` if needed.
- **Driver board.** The outputs are 3.3 V and a few mA. The camera inputs need the driver board
  (#561 step 3). Check the pulse width the cameras need (`CS_TRIG_PULSE_US`).
- **Strobe levels.** Camera strobe outputs are usually open-collector: the driver board must pull
  them up to 3.3 V. Leave unwired inputs pulled down (`CS_STROBE_PULL`) or set them to `CS_NO_PIN`.
- **HSE.** `BOOT` should say `.hse`. `.hsi` means the ST-LINK MCO is not reaching PH0. Pulses
  stay PPS-locked either way, but holdover and free-running are less accurate.
- **Network.** Reserve 10.8.0.60 in `src/interfacing/INTERFACING_README.md` and check the car's
  firewall for UDP 5800 (node open items A2-A4).
- **Scope check.** 50.000 ms period, all outputs together, pulse 0 on the PPS edge, and trigger
  timing unchanged with Ethernet unplugged and replugged.

## Differences from issue #561

- **Pins are set in `config/cs_pins.h`, not in a CubeMX `.ioc`.** The schematic isn't final and
  the same map has to work on two chips, so the pins live in one header with build-time checks.
  The CubeIDE project (`.ioc`, `Core/`) is replaced by a CMake build; any IDE can load the `.elf`.
- **Clock** (#561 step 4): HSE from the ST-LINK MCO with PLL, falling back to HSI automatically.
- **Reporting** (#564) is over both UDP (what the node reads today) and the ST-LINK USB serial
  (issue #564's original plan, and the node's open item F1).
