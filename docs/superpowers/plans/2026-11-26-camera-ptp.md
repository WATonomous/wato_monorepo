# GPS-to-STM32 PTP and camera trigger timestamps Implementation Plan

**Goal:** Synchronize STM32 camera trigger timing to the PwrPak7 over PTP and publish trustworthy
absolute pulse timestamps through the existing ROS camera trigger node.

**Architecture:** The GPS receiver is the intended PTP grandmaster. A PTP client on the STM32
disciplines an Ethernet hardware clock; a measured mapping between that clock and TIM2 schedules
the trigger grid and timestamps the output. The board reports each pulse and its synchronization
quality to `camera_trigger_node`, which publishes it for frame association.

**Tech stack:** NovAtel OEM7, IEEE 1588 PTPv2, STM32 F767/H753 C firmware, ROS 2 C++.

**Deployment target:** NUCLEO-F767ZI over Ethernet. H753 remains an existing build target, but is
not a prerequisite for the first F767 deployment. The vendored F7 HAL has PTP clock and RX/TX
timestamp APIs guarded by `HAL_ETH_USE_PTP`; the active Ethernet port does not use them yet.
Their behavior and accuracy still require hardware verification.

**Spec:** [GPS-referenced PTP target](../../../src/embedded/camera_sync/docs/ptp_time_sync.md).

## What the GPS provides

- Car receiver reported by the operator: PWRPAK7-E1, OEM7700-1.01, firmware
  `OM7CR0902RN0000` (7.09.02). The observed settings are `PTPMODE DISABLE`,
  `PTPPROFILE PTP_UDP`, `PTPTIMESCALE PTP_TIME`. A 30-second capture contained no PTP packets;
  at that time `BESTPOS` was `INSUFFICIENT_OBS` and receiver clock status was `INVALID`.
  `PTPSTATUSA` is not supported by this firmware. These observations do not establish PTP output
  behavior or timing accuracy; capture again after PTP is enabled and the receiver has valid time.
- An indoor capture after enabling PTP shows `10.8.0.8` (clock identity
  `002166fffe04a1ec`) sending PTPv2 domain 0 Delay_Req messages. The capture also shows
  `129.97.181.179` (`1a4b23fffe1763db`) advertising Announce, Sync and Follow_Up with priority1
  10, matching the operator's earlier car-computer `ptp4l --masterOnly 1 --priority1 10` test.
  There are no PwrPak7 Announce or Sync messages in this capture. It is consistent with a
  competing car-computer master, but does not establish the receiver's exact state or prove GPS
  time validity. Capture again with the test master stopped; verify `TIME` and `PTPDELTATIME`.
  The `-i any` capture observes packets on multiple interfaces, so repeated records are not proof
  of duplicate packets on the wire. No Delay_Resp was observed in this capture.
- NovAtel documents PTP support for PwrPak7 and says it was introduced in OEM7 7.09.00. Check
  `VERSION` on the installed receiver: model, firmware and feature availability remain unverified.
  [OEM7 release notes](https://docs.novatel.com/OEM7/Content/New_In_OEM7/New_For_OEM7_09_00.htm)
- Use the [OEM7 7.09.02 Commands and Logs manual](https://docs.novatel.com/OEM7/Content/PDFs/PreviousVersions/OEM7_Commands_Logs_Manual_D1.pdf),
  sections 2.121–2.123, as the version-specific reference. NovAtel labels it `OM7MR0902RN0000`;
  the reported receiver string is `OM7CR0902RN0000`, so confirm any variant-specific differences.
- `PTPMODE ENABLE`, `PTPPROFILE PTP_UDP`, and `PTPTIMESCALE PTP_TIME` are the proposed receiver
  settings to verify on a bench; PTP is disabled by default. The 7.09.02 default UDP profile
  uses multicast, end-to-end delay requests, domain 0, 1 Hz Sync and 0.5 Hz Announce. All nodes
  must agree on profile and master selection. Do not save settings or change the car's GPS
  configuration until its current configuration and downstream effects have been reviewed.
  The 7.09.02 profile table does not specify a Delay_Req interval; measure it on the receiver.
- `PTPMODE ENABLE_FINETIME`, documented for 7.09.02, delays startup until
  FINESTEERING but continues PTP if that status is later lost. It is not a substitute for ongoing
  GPS quality monitoring. The receiver's `TIME` and `PTPDELTATIME` logs provide time quality,
  UTC information and PTP state; confirm the fields and transport available to the ROS driver.
  [TIME](https://docs.novatel.com/OEM7/Content/Logs/TIME.htm),
  [PTPDELTATIME](https://docs.novatel.com/OEM7/Content/Logs/PTPDELTATIME.htm)
- PTP event packets are the source of clock synchronization: a timestamp in a GPS log or the
  host's TCP reception time alone cannot remove variable network delay. `gps.launch.yaml` uses
  the separate OEM7 TCP stream at `10.8.0.8:3001`; PTP must reach the STM32 through the sensor
  network independently of that stream. PTP-aware network devices improve accuracy; multicast
  forwarding and the actual switch behavior must be measured.

## Constraints and design decisions

- Keep TIM2 as the trigger timebase and preserve its high-priority output interrupt. Do not assume
  the Ethernet PTP clock and TIM2 are already synchronized. Prefer hardware RX/TX packet
  timestamps; [ST's H753 manual](https://www.st.com/resource/en/reference_manual/rm0433-stm32h742-stm32h743-753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
  documents IEEE 1588 timestamp support. Independently verify the F767, HAL and DMA descriptor
  paths on the F767. If hardware timestamping or a sufficiently accurate clock mapping is not
  available, stop and revise the error budget/design rather than claiming equivalent accuracy.
- Use PTP time internally and carry the applicable UTC offset and its validity to the ROS boundary.
  Never hardcode the current offset. Define leap-second behavior and reject stale or invalid UTC
  metadata before publishing a valid UTC stamp. The GPS `TIME` log's GPS-to-UTC offset is not
  automatically interchangeable with the PTP-to-UTC offset.
- Version 1 retains its existing PPS meaning. Version 2 reports an absolute timestamp and timing
  validity; never set `pps_locked` to mean PTP lock. Define a coordinated message migration for
  consumers because ROS message schema changes also change the type contract.
- For every `time_valid` pulse, the published UTC timestamp must be within 2.5 ms of its physical
  trigger edge in the GPS-referenced time domain. Budget synchronization, timer mapping, output
  latency and time-scale conversion within that bound, including uncertainty; reject timestamps
  when the bound cannot be established. A 20 Hz pulse train has a 50 ms period, not 5 ms.
- Exposure-skew, holdover and recovery limits are not yet defined. Agree and record them before
  final hardware acceptance; do not mistake the prior 1 µs board-output skew criterion for an
  exposure-skew requirement.

## Tasks

### 1. Verify source and network

**Files:** `src/interfacing/sensor_interfacing/launch/gps.launch.yaml`,
`src/interfacing/sensor_interfacing/config/novatel_oem7_overrides.yaml`,
`src/interfacing/sensor_interfacing/docs/gps_bringup.md`.

- [ ] On a bench, record GPS `VERSION`, existing PTP settings and `TIME` status; check profile
  support and enable the proposed configuration temporarily. Capture PTP traffic at the host and
  STM32 switch port. Record grandmaster identity, one/two-step behavior, domains, transport,
  message intervals, correction fields, UTC-offset flags and delay response behavior.
- [ ] Confirm the switch forwards PTP multicast, assess whether it is PTP-aware, and verify both
  endpoints can receive the GPS packets. Exercise loss of GNSS and record how quality flags change.
- [ ] Document repeatable receiver commands and resulting configuration. Add startup configuration
  only after the bench results establish it is compatible with the existing GPS and lidar setup;
  preserve the existing PPS output used by lidar.
- [ ] Pass criterion: packet capture proves the receiver is the chosen grandmaster and exposes
  the time-scale and quality information needed by the implementation. If not, resolve that
  prerequisite before proceeding; do not infer it from normal GPS log timestamps.

### 2. Ethernet timestamping and PTP client

**Files:** `src/embedded/camera_sync/app/src/cs_net.c`, `app/include/cs/cs_net.h`,
`port/stm32/eth_port.c`, `port/stm32/eth_port.h`, `tests/test_net.c`; add focused PTP modules and
host tests under `app/` and `tests/`, and register them in `cmake/host_tests.cmake`.

- [ ] Verify F767 RX and TX hardware timestamps with a bench packet capture and the relevant reference
  manual. Extend receive filtering beyond the present unicast/broadcast
  and ICMP-only path to accept the selected PTP multicast UDP traffic; retain ARP, ping and reports.
- [ ] Select a maintained embedded PTP implementation compatible with the actual profile and
  resource budget, or justify a minimal profile-specific implementation. Add timestamped packet
  ingress/egress, Sync/Follow_Up as observed, Announce, Delay_Req/Delay_Resp, correction fields,
  matching by identity/sequence, and timeout handling. Reject malformed, stale, wrong-domain or
  wrong-master messages. Implement and test actual device transmit timestamps, not enqueue time.
- [ ] Test both one- and two-step messages if supported, sequence rollover, reordering, duplicates,
  correction fields, lost packets, master changes, multicast acceptance and existing UDP reporting.
  Run `cd src/embedded/camera_sync && cmake --preset host && cmake --build --preset host && ctest --preset host`.
- [ ] Pass criterion: the board follows the intended GPS grandmaster and measured PTP-clock error
  stays within its allocated budget over representative network load and reconnects.

### 3. Discipline TIM2 and produce timed pulses

**Files:** `app/src/cs_sched.c`, `app/include/cs/cs_sched.h`, `port/stm32/trigger.c`,
`config/cs_config.h`, `tests/test_sched.c`; add clock-mapping tests as needed.

- [ ] Create a bounded-error mapping from the disciplined Ethernet clock to the wrapping TIM2
  counter using hardware correlation where available, otherwise bracketed reads with measured
  uncertainty. Update its rate and phase without racing the timer interrupt; reject stale or
  ambiguous mappings. Schedule each 50 ms grid instant in TIM2 ticks, with no direct PPS input.
- [ ] Capture the timer count immediately after the GPIO BSRR write and associate it with the
  pulse record; transform it using a consistent clock-map snapshot. Include the calibrated
  count-read/output latency and uncertainty, or explicitly report the scheduled time with an
  independently measured bound. Never claim the scheduled grid instant is the measured edge.
- [ ] Define acquisition, lock, holdover and fault transitions, bounded corrections, safe handling
  of backwards steps and missed deadlines, and re-acquisition. Report invalid on startup, stale
  time, invalid mapping or loss of the agreed bound. Simulate drift, wrap, delayed updates, clock
  steps and PPS-free operation; verify no duplicate or late pulses and no data races.
- [ ] On the F767 compare the output with an independent GPS-referenced measurement
  under network and CPU load. Pass only if scheduling, mapping and GPIO latency fit their allocated
  error budget; host simulations do not establish hardware timing accuracy.

### 4. Versioned reports and ROS publication

**Files:** `app/include/cs/cs_report.h`, `app/src/cs_report.c`, `tests/test_report.c`,
`docs/report_protocol.md`, `src/perception/camera_trigger/src/{protocol.cpp,camera_trigger_node.cpp}`,
`src/perception/camera_trigger/include/camera_trigger/`,
`src/perception/camera_trigger_msgs/msg/CameraTrigger.msg` and associated tests and docs.

- [ ] Specify version 2 fields: boot/session identity, sequence, pulse timestamp in PTP seconds
  and nanoseconds, clock state, time/offset validity, UTC offset and its validity, synchronization
  age and uncertainty, and strobe data. Specify integer widths, ranges, checksum, timestamp
  meaning, step/reboot behavior and invalid-field semantics. Treat field layout as a coordinated
  firmware/host contract, and leave version 1 decoding and PPS semantics intact during migration.
- [ ] Serialize and parse v2 with tests for invalid fields, rollover, reboot, packet loss,
  duplicates, ordering, invalid UTC metadata and old v1 reports. Add a compatible ROS migration
  path: use a new message/topic or coordinate all consumers before changing the existing message.
- [ ] Convert the board pulse time to ROS UTC only when its time scale and offset are valid; use
  board lock, mapping quality, reference health and age to set `time_valid`. Keep receive time
  as a diagnostic, not the timestamp source. Reject stale GPS quality information; a host-only
  check must not silently mark pulses valid when the host or GPS health feed is absent.
- [ ] Test the node against a fake v2 board: exact timestamp survives arbitrary permitted packet
  latency, invalid lock/UTC/reference health is invalid, and recovery, reboot and loss are visible
  in `/diagnostics`. Preserve existing v1 tests and behavior.

### 5. Integrate cameras and acceptance

**Files:** `src/perception/camera_trigger/docs/open_items.md`,
`src/perception/perception_bringup/launch/perception.launch.yaml`, bag profiles, and external
`WATonomous/camera_aravis2_nitros` camera driver.

- [ ] Synchronize the host to the same PTP reference with one documented clock owner. Verify
  PTP-to-system-clock UTC conversion and GPS health delivery to the ROS node; test stale and
  conflicting services. Add the node, message types, diagnostics and pulse topic to car launch
  and recording configuration.
- [ ] Match camera frames to pulse sequence/timestamps; account for frame drops, ambiguity and
  trigger-to-exposure delay, and propagate invalid timing. Check driver support for every image
  publish path, and measure camera strobe/exposure timing instead of assuming GPIO simultaneity.
- [ ] Record the 2.5 ms timestamp limit, remaining agreed thresholds and equipment, then run a sustained valid-GPS test under
  load, independent trigger/exposure measurements, and loss/recovery tests for GNSS, PTP, network,
  host, board and cameras. Record absolute error, inter-camera skew, valid/invalid transitions,
  packet gaps and recovery time. Ship only when all measured bounds meet the agreed limits.
