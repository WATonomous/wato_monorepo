# Trigger board report protocol, version 1

The contract between the STM32 camera trigger board firmware and `camera_trigger_node`.
This file is the single source of truth on the computer side: if the firmware changes, this
document and the parser (`src/protocol.cpp`, `test/test_protocol.cpp`) change together.

> **Status.** Written from the computer-side specs (`CAMERA_TRIGGER_COMPUTER_SIDE.md` and its
> Ethernet delta). It still has to be cross-checked against the firmware's own contract,
> `FIRMWARE_F767ZI_ETHERNET.md` section 6 ([open item A1](open_items.md#a-information-to-collect)).
> The file name is kept from the original serial design; the board now reports over Ethernet.

## Link

| | |
|---|---|
| Board | NUCLEO-F767ZI |
| Transport | UDP over Ethernet, IPv4, from the board to the computer |
| Destination port | 5800 (`udp_port`) |
| Framing | one datagram = one line |
| Maximum line length | 256 bytes, terminator included |
| Character set | printable ASCII (0x20-0x7E), then `\r\n` |

The board's static IP and the computer's destination address are set in one place on each side:
the firmware config header, and `config/camera_trigger.yaml` (`udp_bind_address`,
`udp_source_ip`). On the car the computer listens on the sensor network, 10.8.0.1 (`br0`).

A USB serial (ST-LINK virtual COM port) fallback with the same lines is planned but not
implemented yet ([open item F1](open_items.md#f-deferred-from-pr-1)).

## Line format

Every line is

```text
<body>*<XX>\r\n
```

`XX` is the XOR of all bytes of `<body>`, as two hex digits (the NMEA scheme). The board sends
uppercase; the node also accepts lowercase. `<body>` is comma-separated fields, the first being
the line type. Lines with a missing or wrong checksum, bad framing, non-ASCII bytes or over
256 bytes are counted as bad lines and dropped.

### `TRG`: one trigger pulse

Sent once per pulse, after a short strobe window following the pulse (a few ms). Sending must
never delay the pulse itself.

```text
TRG,1,<seq>,<pps_count>,<pulse_idx>,<pps_locked>,<strobe_mask>*XX
```

| Field | Format | Meaning |
|---|---|---|
| `TRG` | text | Line type |
| `1` | decimal | Protocol version. The node rejects other versions with a clear error. |
| `seq` | uint32, decimal | +1 on every pulse. 0 after the board boots. Wraps at 2^32. |
| `pps_count` | uint32, decimal | Seconds counted by the board since boot: +1 on each PPS edge, or every `pulses_per_second` pulses while PPS is lost. |
| `pulse_idx` | decimal, 0 to `pulses_per_second` - 1 | Pulse index within the current second. 0 is the pulse fired on the PPS edge. |
| `pps_locked` | `0` or `1` | 1 if the pulse train is currently locked to PPS. |
| `strobe_mask` | 3 hex digits, or `---` | Bit k set = strobe input STB(k+1) saw a camera start exposing for this pulse. `FFF` = all 12. `---` if the firmware doesn't measure strobes. |

Numbers are plain decimal: no sign, spaces or `0x` prefix.

### `BOOT`: the board started

Sent once when the board starts. The node resets its sequence and timing state.

```text
BOOT,1,<fw_version>*XX
```

| Field | Format | Meaning |
|---|---|---|
| `fw_version` | non-empty text without commas | Firmware version string |

### Other line types

Lines of any other type (for example `STAT`, planned for firmware health:
[open item F2](open_items.md#f-deferred-from-pr-1)) are counted and ignored, so newer firmware can
add line types without breaking the node.

## Examples

Real checksums:

```text
BOOT,1,v1.0.0*60
TRG,1,1234,61,14,1,FFF*01
TRG,1,1235,61,15,1,FFB*05
TRG,1,1236,61,16,0,---*6B
```

The second `TRG` line: STB3 (bit 2) saw no exposure for pulse 1235. The third: PPS lost,
strobes not measured.

## How the computer times a pulse

The node never trusts its own receive time as the pulse time. For a locked pulse received at
`t_rx` on the GPS-synced system clock, with period `T = 1 / pulses_per_second`:

1. `second = round(t_rx - expected_latency - pulse_idx * T)`, and
   `pulse_time = second + pulse_idx * T`.
2. `residual = t_rx - pulse_time` must be between 0 and `max_latency`, otherwise the stamp is
   marked `time_valid = false` (the system clock isn't synced, or the report is stale).
3. `second - pps_count` must stay constant. One line that disagrees is marked invalid; a new value
   is adopted after 20 consecutive lines agree (board reboot, clock correction).
4. Unlocked pulses get `t_rx - expected_latency` and `time_valid = false`.

So the report delay doesn't need to be fixed or known exactly, as long as it stays well under
half a second. The firmware only has to send `pulse_idx` and `pps_count` correctly.

## PPS input

The board locks to the PwrPak7's PPS output. The receiver's saved PPS polarity was NEGATIVE
on 2026-11-17 and only becomes POSITIVE when the ROS GPS driver connects (#560). If the board
triggers on the wrong edge, every pulse is off by the PPS pulse width (1 ms), which the computer
cannot detect ([open item A9](open_items.md#a-information-to-collect)).
