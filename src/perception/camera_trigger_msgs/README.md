# camera_trigger_msgs

Message types for the camera trigger system. Published by the
[`camera_trigger`](../camera_trigger/README.md) node.

## Messages

### CameraTrigger.msg

One hardware trigger pulse reported by the STM32 trigger board. Published on `/camera_trigger`,
one message per pulse (20 Hz). Pulses that were never reported are never published.

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | `stamp` = time the pulse fired, on the system clock (UTC). `frame_id` = `camera_trigger` |
| `seq` | `uint32` | Pulse counter from the board. +1 per pulse, 0 after a board reboot, wraps at 2^32 |
| `pps_count` | `uint32` | Seconds counted by the board since boot |
| `pulse_idx` | `uint8` | Pulse index within the current second; 0 = the pulse fired on the PPS edge |
| `pps_locked` | `bool` | The board's pulse train is locked to the GPS PPS signal |
| `time_valid` | `bool` | The stamp was snapped to the PPS grid and passed every check |
| `strobe_mask` | `uint16` | Bit k = strobe input STB(k+1) saw a camera start exposing for this pulse |
| `strobe_mask_valid` | `bool` | False if the board does not measure strobes |
| `receive_time` | `builtin_interfaces/Time` | System time the report was received |

`time_valid` assumes the system clock is synced to GPS (chrony, issue #560). A clock error of a
whole number of seconds cannot be detected from the board's reports.
