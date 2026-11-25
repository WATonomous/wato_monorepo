# Trigger board report protocol, version 1 (firmware side)

What the camera trigger firmware sends to the computer. The computer side is
`camera_trigger_node` (PR #611, `src/perception/camera_trigger/docs/serial_protocol.md`). This file is
the firmware contract that document's open item A1 asks for. Change both documents, the firmware
formatter (`app/src/cs_report.c`, `tests/test_report.c`) and the node parser together.

## Transports

The same lines go out on both transports. Either can be turned off in `config/cs_config.h`.

| | UDP | USB serial |
|---|---|---|
| Path | NUCLEO Ethernet port to the sensor network | ST-LINK virtual COM port (USART3), `/dev/ttyACM*` on the computer |
| Framing | one datagram = one line | byte stream; split on `\r\n` |
| Addresses | from 10.8.0.60:5800 to 10.8.0.1:5800 (`CS_NET_*`) | 115200 baud, 8N1 |
| Starts | once the computer answers ARP (lines before that are not sent over UDP) | at power-up |

The board answers ARP and ping on its address, so `ping 10.8.0.60` works. It has no DHCP: the
address is static and must be outside the dnsmasq range (10.8.0.100-200).

## Line format

```text
<body>*<XX>\r\n
```

`XX` is the XOR of every byte of `<body>`, as two uppercase hex digits. Lines are printable ASCII,
at most 256 bytes including `\r\n`. Fields are comma-separated; the first is the line type, the
second the protocol version (`1`).

### `BOOT`: the firmware started

```text
BOOT,1,<fw_version>*XX
```

`fw_version` is `<version>+<board>.<clock>`, for example `1.0.0+h753zi.hse`. `clock` is `hse`
(PLL from the ST-LINK's 8 MHz crystal-derived clock) or `hsi` (internal RC fallback, about 1 %
accuracy; check the board's solder bridges if you see it).

Sent once per boot: on the UART at power-up, and over UDP as soon as the computer answers ARP.

### `TRG`: one trigger pulse

```text
TRG,1,<seq>,<pps_count>,<pulse_idx>,<pps_locked>,<strobe_mask>*XX
```

| Field | Meaning |
|---|---|
| `seq` | +1 per fired pulse, 0 for the first pulse after boot, wraps at 2^32 |
| `pps_count` | second number of the pulse: +1 every second, starting at 0 at boot |
| `pulse_idx` | 0 to 19: position in the second. Pulse 0 is the pulse on the PPS edge |
| `pps_locked` | 1 if the pulse train was locked to PPS when this pulse fired |
| `strobe_mask` | 3 hex digits: bit k set = strobe input STB(k+1) saw an edge for this pulse. `---` if no strobe input is configured |

Sent when the pulse's strobe window closes (`CS_STROBE_WINDOW_US`, 5 ms after the pulse), so a
report arrives about 5 ms + network delay after its pulse. That is inside the node's 0-100 ms
residual window. Its `expected_latency` default of 10 ms also works; set it to the measured value.

How the fields behave:

- **Locked** (`pps_locked = 1`): pulse `pulse_idx` of second `pps_count` fired at
  `PPS edge + pulse_idx * 50 ms`. `pps_count` advances exactly once per PPS edge.
- **Lock acquired** (after 2 valid 1 s PPS intervals): the train jumps to the PPS phase.
  `pps_count` increases by 1, `pulse_idx` restarts at 1 or later (slot 0 was the edge itself,
  already past), and `seq` continues. The node re-learns its second offset (about 1 s).
- **PPS lost** (no valid edge for 1.5 s): `pps_locked = 0`, and the train continues on the last
  measured rate (holdover), so pulses stay within microseconds of the PPS grid for many seconds.
  When PPS returns on the same phase, the train relocks without a jump or renumbering.
- **No PPS since boot**: free-running at the nominal rate, `pps_locked = 0`.
- A pulse that could not be fired on time is skipped, never fired late. `seq` counts only fired
  pulses, so a skip shows as a `pulse_idx` jump without a `seq` gap. It is counted in `STAT`.

### `STAT`: firmware health, once per second

```text
STAT,1,<uptime_s>,<state>,<pps_err_ns>,<pps_glitches>,<skipped_pulses>,<stray_strobes>,<tx_drops>*XX
```

Sent right after the `TRG` line of every `pulse_idx = 0` pulse.

| Field | Meaning |
|---|---|
| `uptime_s` | seconds since boot |
| `state` | `LOCKED`, `HOLDOVER` (PPS lost, holding the last rate) or `FREERUN` (no lock since boot) |
| `pps_err_ns` | signed: last PPS edge minus the predicted second start (pulse 0 fires at the prediction). `0` until a locked edge has been measured |
| `pps_glitches` | total PPS edges rejected while locked (more than 100 us off the prediction) |
| `skipped_pulses` | total pulses not fired because their time had already passed |
| `stray_strobes` | total strobe edges outside every strobe window |
| `tx_drops` | total report lines lost: queue full, UART buffer full, or an Ethernet send error |

All counters are totals since boot. This is the only signed field (`-` prefix). The node ignores
`STAT` today (counted as an unknown line type). Parsing it is the node's open item F2.

## Verified against the node

The host test `tests/test_report.c` checks the formatter byte for byte against the examples in the
node's `serial_protocol.md`. A one-off cross-check also ran the firmware's scheduler and formatter
through the node's own `parseLine` and `PulseTimer` (PR #611 sources): 900 `TRG` lines over 45 s
with a 10 s PPS dropout, with no parse or field mismatches. Locked pulses were `time_valid`, and
unlocked pulses never were.
