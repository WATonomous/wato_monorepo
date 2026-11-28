# camera_trigger

Publishes every hardware camera trigger pulse with the exact time it fired.

The STM32 trigger board (NUCLEO-F767ZI) fires a PPS-locked 20 Hz pulse into all Hikrobot cameras
over wires, then reports each pulse to the computer as one UDP datagram. `camera_trigger_node` turns
each report into a `camera_trigger_msgs/CameraTrigger` message on `/camera_trigger`, stamped with
the pulse time on the system clock, and reports its health on `/diagnostics`.

The camera driver will use `/camera_trigger` to stamp every frame with the pulse that captured it
(issue #563 step 3, not done yet). On its own, `/camera_trigger` already shows whether the trigger
system works and when each pulse fired.

```text
PwrPak7 GPS --PPS--> STM32 trigger board --trigger wires--> cameras
                          |
                          | UDP, one line per pulse (port 5800)
                          v
               camera_trigger_node --> /camera_trigger (one message per pulse, 20 Hz)
               (perception container,   /diagnostics    (health, 2 Hz)
                own process)
```

Related docs:

- [GPS-referenced PTP target](../../embedded/camera_sync/docs/ptp_time_sync.md): planned replacement for PPS synchronization
- [docs/serial_protocol.md](docs/serial_protocol.md): the report protocol shared with the firmware
- [docs/open_items.md](docs/open_items.md): unconfirmed items and next steps
- [DEVELOPING.md](DEVELOPING.md): code layout, building and testing

## Topics

| Topic | Type | Notes |
|---|---|---|
| `/camera_trigger` | `camera_trigger_msgs/CameraTrigger` | One message per reported pulse. Reliable, keep last 100. Pulses that were never reported are never published. |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | One status named `camera_trigger`, 2 Hz |

The message fields are described in
[camera_trigger_msgs](../camera_trigger_msgs/README.md). The important one is `time_valid`: only
then is `header.stamp` the exact pulse time.

## How a pulse is timed

PPS second boundaries are the same in GPS time and UTC, so once the system clock is synced to GPS
(chrony, issue #560), pulse `pulse_idx` of a second fired exactly at
`second + pulse_idx / 20`. The node finds the second by rounding the receive time (taken by the
kernel when the datagram arrived), so the network delay doesn't affect the stamp. Then:

- the delay between pulse and report must be between 0 and `max_latency` (100 ms), otherwise the
  clock isn't synced (or the report is stale): `time_valid = false` and a warning;
- the board's second counter must agree with the clock's seconds; the agreement is learned from the
  first 20 pulses (about 1 s, during which stamps are `time_valid = false`) and re-learned after a
  board reboot;
- a pulse not locked to PPS gets an approximate stamp and `time_valid = false`.

## Parameters

Defaults are in [config/camera_trigger.yaml](config/camera_trigger.yaml).

| Parameter | Default | Meaning |
|---|---|---|
| `udp_bind_address` | `0.0.0.0` | Local IPv4 address to listen on. On the car: `10.8.0.1` (sensor network, `br0`). |
| `udp_port` | `5800` | Must match the firmware's destination port. |
| `udp_source_ip` | `""` | Accept reports only from this address (the board's IP). Empty accepts any sender. |
| `pulses_per_second` | `20` | |
| `expected_latency` | `0.010` s | Typical delay between a pulse and its report arriving. Used for rounding to the right second. |
| `max_latency` | `0.100` s | A larger delay (or a negative one) marks the stamp invalid: clock not synced, or stale report. |
| `stale_timeout` | `0.3` s | No report for this long: ERROR. |
| `reconnect_period` | `1.0` s | First retry delay if the socket can't be opened (doubles, up to 10 s). |
| `strobe_miss_warn_count` | `20` | Pulses in a row without a strobe before an input is reported. |
| `expected_strobe_bits` | `[0..11]` | Strobe inputs with a camera wired (0 = STB1 ... 11 = STB12). Unwired inputs never strobe, so leave them out. |

Invalid values (for example a malformed IP address) stop the node at startup with a clear message.

## Diagnostics

One status, `camera_trigger`, at 2 Hz. Each condition appears within a second and clears by itself.

| Level | Condition |
|---|---|
| ERROR | Socket can't be opened (for example port in use); no report for `stale_timeout` |
| WARN | PPS not locked |
| WARN | Pulse reports missing in the last 10 s (count) |
| WARN | Pulse rate not 20 +/- 0.5 Hz over the last 2 s |
| WARN | Reports arrive outside 0 to `max_latency` after their pulse: system clock not synced to GPS (#560) |
| WARN | A strobe input in `expected_strobe_bits` missing for `strobe_miss_warn_count` pulses in a row |
| WARN | Bad report lines (checksum, framing, fields, version, size) in the last 10 s (count) |

Key/values include the firmware version, sender address, rate, pulses received and published,
last seq, last residual, totals for gaps, duplicates, late and bad lines, and per-input strobe misses.

## Running

```bash
ros2 launch camera_trigger camera_trigger.launch.yaml
# or with another parameter file:
ros2 launch camera_trigger camera_trigger.launch.yaml config_file:=/path/to/params.yaml
```

The node runs as its own process, not inside `perception_container`, so a fault in it cannot stop
the camera drivers.

### Without hardware: the fake board

`tools/fake_stm32.py` (Python, no dependencies) sends the same reports as the real board, 20 times
per second on the PPS grid of this computer's clock:

```bash
ros2 run camera_trigger fake_stm32.py                       # to 127.0.0.1:5800
ros2 run camera_trigger fake_stm32.py --udp 10.8.0.1:5800   # to another address
```

Fault options, for checking that the node notices each problem:

| Option | Simulates | Expected in `/diagnostics` |
|---|---|---|
| `--drop-seq N` | Every N-th report lost | WARN: reports missing. The missing seq numbers are never published. |
| `--pause S` (`--pause-at T`) | Cable unplugged for S seconds | ERROR within 1 s, then WARN for the missed pulses, then OK |
| `--unlock-after S` | PPS lost | WARN: PPS not locked; `time_valid = false` |
| `--clock-offset S` | Computer clock off by S seconds | WARN: clock not synced, when \|S\| moves the delay out of 0-100 ms |
| `--reboot-after S` | Board reboot | Short ERROR, then OK. Stamps valid again about 1 s later. |
| `--duplicate-rate P` | Duplicated packets | `duplicates_total` rises; nothing is published twice |
| `--garble-rate P` | Corrupted lines | WARN: bad report lines (`bad_checksum`) |
| `--strobe-miss CAM:RATE` | Camera CAM (0-11) not exposing | WARN for STB(CAM+1) when it misses 20 pulses in a row |
| `--no-strobe` | Firmware not measuring strobes | `strobes = not measured by the board` |
| `--latency S`, `--latency-jitter S` | Report delay | `last_residual_ms` follows |

Run `fake_stm32.py --help` for all options. If the fake itself falls behind (busy machine), it
prints `line sent ... ms late`.

To watch: `ros2 topic hz /camera_trigger`, `ros2 topic echo /camera_trigger`, and
`ros2 topic echo /diagnostics` (or the Foxglove diagnostics panel).

### Bench setup (laptop and board)

1. Connect the board's Ethernet port straight to the laptop.
2. Give the laptop's port a static IP on the board's subnet, the address the firmware sends to.
3. Check raw reports arrive: `nc -ul 5800` should print a `TRG` line 20 times per second.
4. Run the node with `udp_bind_address` set to that IP (or `0.0.0.0`).

### On the car

- The board plugs into the sensor network switch (`br0`, 10.8.0.0/16). Set `udp_bind_address` to
  `10.8.0.1` and `udp_source_ip` to the board's IP.
- The perception container uses host networking, so no Docker port mapping is needed.
- If a host firewall is active, allow the reports:
  `sudo ufw allow from <board IP> to any port 5800 proto udp`.
- Stamps are only exact once the system clock is synced to GPS (#560).

## Limitations

- **The system clock must be synced to GPS** (chrony, #560). An error of a whole number of seconds
  cannot be detected: the board only counts seconds since its own boot.
- **Without a GPS fix** (for example indoors), the receiver's PPS isn't aligned to UTC, so stamps
  can look valid while being wrong. Hardware timing checks need the car outside.
- **PPS polarity:** if the board triggers on the wrong PPS edge, every pulse is off by the PPS pulse
  width (1 ms), which the node cannot see. See [serial_protocol.md](docs/serial_protocol.md).
- UDP only for now; the USB serial fallback is open item F1.
