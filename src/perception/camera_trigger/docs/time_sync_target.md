# GPS-referenced time synchronization target

**Status: target architecture, not yet implemented or validated end to end.** The goal is to
use GPS-referenced PTP to align the host and STM32 timing, and to assign each camera frame the
time of its actual capture. PTP lock alone does not establish that a camera exposed at the time
in its image header.

## Timing path

```text
GPS receiver / GPS-referenced PTP grandmaster
    | PTP over sensor network                 | GPS PPS (if used)
    +--> host clock                           +--> STM32 trigger timing
    +--> STM32 clock (if supported)                     |
                                                       +--> camera trigger inputs
                                                       +--> pulse reports --> host --> frame stamps
```

- Establish a GPS-referenced PTP grandmaster and synchronize the host clock and the STM32's
  timing reference to it. Confirm whether the receiver itself supplies PTP; otherwise a separate
  GPS-disciplined grandmaster is needed. Confirm PTP support, timestamping capabilities,
  network configuration, profile and time scale on every participating device.
- If the STM32 uses a direct GPS PPS input for the trigger grid, verify that the PPS and PTP
  grandmaster share the same reference and phase. If it instead schedules triggers from its
  PTP-synchronized clock, verify the accuracy of the clock-to-output scheduling path. The
  implementation and evidence must identify which approach is used; PTP on the network alone
  does not synchronize the board.
- Trigger all cameras from the same board. Associate each frame with its actual trigger, accounting
  for missed triggers, dropped frames, ordering and camera exposure/trigger delay. Publish a
  timestamp in the same time domain as other sensor data and carry timing validity through to
  consumers. The cameras need not support PTP if their capture times are established by this
  validated hardware-trigger path.
- Define conversion between PTP time and the host/ROS timestamp time scale, including UTC offset
  and leap-second handling. Do not assume a PTP timestamp is already a Unix/UTC timestamp.

## Validity and failure handling

Define an end-to-end timing error budget and acceptable recovery time before deployment. Include
grandmaster/GPS quality, network and clock synchronization error, STM32 trigger scheduling,
electrical and camera exposure delays, and frame-to-trigger association. Measure both absolute
error against an independent reference and relative exposure skew across cameras.

Monitor GPS time validity, grandmaster and PTP synchronization state, offset and age of the last
good synchronization, STM32 trigger lock, and frame-to-trigger association. Loss of reference,
clock steps, reboot, stale reports, or an ambiguous association must make affected timestamps
invalid until synchronization and association are re-established. Define and test any holdover
policy against the error budget; a running clock alone is not proof of valid time.

## Current state and work remaining

The existing host node reconstructs timestamps from a reported PPS pulse index and a host clock
assumed to be GPS-synchronized; it does not check PTP or GPS time quality. Its `time_valid` flag
therefore cannot by itself prove the target guarantees. See [README](../README.md) and
[protocol](serial_protocol.md). The checked-in STM32H753 sample uses an internal oscillator and
free-running PWM; it does not implement the documented PPS or PTP path. The host documents a
different NUCLEO-F767ZI board, whose firmware contract still needs confirmation.

The repository's open items also leave camera frame stamping, car integration, PPS polarity and
hardware acceptance outstanding. Chrony over NMEA is recorded as the current host setup, not
evidence that the proposed PTP infrastructure is active. See [open items](open_items.md).

Before calling the target complete:

1. Identify the actual GPS/PTP grandmaster, STM32 hardware and deployed firmware; verify their
   capabilities and chosen reference path. Configure and monitor synchronization on each clock.
2. Implement or verify deterministic, reference-aligned trigger generation and pulse reporting;
   confirm the trigger rate and electrical edge on the actual board.
3. Implement camera frame association and validity propagation, and integrate the node into the
   car's launch, diagnostics and recordings.
4. With a valid GPS fix, measure clock offsets and trigger/exposure timing over a representative
   run against the agreed error budget. Test startup, GPS/PTP loss and recovery, clock steps,
   board reboot, delayed or lost reports, and dropped frames. Record equipment, results and
   pass/fail limits.
