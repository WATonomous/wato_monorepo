# Camera trigger: open items and next steps

What is still unconfirmed or left for later in the camera trigger work (#564, #563 step 3), how to
get each item, and where it plugs in. Keep this file current: close items as they're done.

The first PR delivers the computer-side trigger node: UDP only, core logic only, tested against
the fake board (`tools/fake_stm32.py`). Frame stamping in the camera driver (section C) and the car
wiring (section D) come later.

## A. Information to collect

| # | Item | Why it's needed | How to get it | Where it goes |
|---|---|---|---|---|
| A1 | **Firmware contract** (`FIRMWARE_F767ZI_ETHERNET.md` section 6) | The node was built from the computer-side specs. The firmware's contract is the source of truth for the port, framing and the `STAT` line. | Ask the firmware owner for the document. | Diff it against [serial_protocol.md](serial_protocol.md). Change the parser (`src/protocol.cpp`) and `test/test_protocol.cpp` together. |
| A2 | **Trigger board static IP** (and MAC) | Source filtering, firewall rule | The firmware owner (it's in the firmware config header). Once the board is on the network: `cat /var/lib/misc/dnsmasq.leases` or `ip neigh show dev br0` on the car. | `udp_source_ip` in `config/camera_trigger.yaml`. Optionally a `dhcp-host` reservation in `src/interfacing/INTERFACING_README.md`. |
| A3 | **Switch port** | The board must be on the sensor network: `br0` (`enp8s0f0` + `enp8s0f1`), 10.8.0.1/16 | On the car: a free port on the NETGEAR switch, then `ping <board IP>`. | README, car setup |
| A4 | **Firewall on the car** | A firewall would silently drop UDP 5800. | `sudo ufw status verbose` and `sudo nft list ruleset`. If one is active: `sudo ufw allow from <board IP> to any port 5800 proto udp`. | README, car setup |
| A5 | **Camera to strobe wiring** (STB1-12 = PC0-PC11 = J3-J14) | Diagnostics name strobe inputs (`STB3`), not cameras. Unwired inputs must be left out of `expected_strobe_bits`. | The people wiring the car. | `expected_strobe_bits` in `config/camera_trigger.yaml`, and a camera-name mapping (F5). |
| A7 | **Real report latency** | `expected_latency` (10 ms) is an estimate. | On the car with the board running: the median of `last_residual_ms` in `/diagnostics` over 10 minutes. | `expected_latency` in the config. Record the value in the PR. |
| A9 | **PPS edge and polarity** | The receiver's saved PPS polarity was NEGATIVE; the GPS driver only sets POSITIVE when it connects (#560). A board triggering on the wrong edge puts every pulse 1 ms off, which the node cannot detect. | #560 (`PPSCONTROL ENABLE POSITIVE` + `SAVECONFIG`). The firmware owner confirms which edge the board uses. | [serial_protocol.md](serial_protocol.md), PPS input |
| A10 | **No GPS fix indoors** | Without a fix the receiver's PPS isn't aligned to UTC. The board can still report `pps_locked=1`, and stamps can look valid while being wrong. | Known limitation. Possible follow-up: also require a valid GPS time status (`/novatel/oem7/bestpos`, `nov_header.time_status`). | README, limitations |
| A11 | **Residual tolerance once chrony runs** | Chrony over NMEA is good to a few ms, about the same as the report latency. A clock a few ms behind GPS gives slightly negative residuals, so valid pulses get marked invalid. | Watch `last_residual_ms` and `last_bad_residual_ms` now that chrony runs (#560, done 2026-11-26). | If needed, a small negative allowance (a `min_latency` parameter). |

## B. Choices made where the specs were silent

Each is one small function with its own unit test, so changing it stays local.

- **Startup:** stamps stay `time_valid=false` until 20 pulses agree on the second offset (about 1 s).
  `PulseTimer`, `src/pulse_timing.cpp`.
- **Late (reordered) packets** up to 10 pulses old are dropped and counted, not published.
  `SequenceTracker`, `src/sequence_tracker.cpp`. Publishing them instead is F3.
- **Unknown line types** (`STAT` until F2) are counted as info, not as bad lines, so newer firmware
  doesn't raise warnings. `countBadLine()`, `src/camera_trigger_node.cpp`.
- **Strobe warnings** only cover the inputs listed in `expected_strobe_bits` (default: all 12),
  because an unwired input never strobes.
- **No `SO_REUSEADDR`:** a second node on the same port fails with "Address already in use"
  instead of silently splitting the reports.
- **Diagnostics are published directly** as `diagnostic_msgs/DiagnosticArray` on `/diagnostics`
  (one status, `camera_trigger`, at 2 Hz) instead of through `diagnostic_updater`. Same message for
  Foxglove and other tools, one dependency fewer.

## C. Next PR: frame stamping in the camera driver

- **Where:** the external repo `WATonomous/camera_aravis2_nitros`, which
  `docker/perception.Dockerfile` clones at unpinned `main`.
  - The hook is `fillImageMsgMetadata()` (`camera_aravis2/src/camera_driver.cpp`). It has one call
    site, before the raw/NITROS split, so one change covers both publish paths.
  - It runs on a per-stream processing thread, while a `/camera_trigger` subscription runs on an
    executor thread: the pulse buffer needs a mutex.
- **Build:** `find_package(camera_trigger_msgs QUIET)` and compile the feature under a
  `WITH_TRIGGER_STAMP` define, like `WITH_NITROS`, so the driver repo still builds on its own.
  `camera_trigger_msgs` is already in the perception image.
- **Problem with the spec's numbers:** aravis takes the system timestamp when a frame's *first*
  packet arrives, so a frame can arrive about 1-3 ms after its pulse. With the spec's
  `trigger_min_latency = 8 ms`, the matcher would lock onto the pulse *before* the right one.
  Use about 0.5 ms and test 1-3 ms latencies.
- **Must detect:** `GevTimestampTickFrequency` reading 0, in which case aravis silently returns
  system time as the device time.
- **Needs hardware:** measure device-timestamp deltas (about 50 ms) on a real camera, with the
  cameras in trigger mode (#563 step 2).

## D. Car wiring PR

- Add the node to `src/perception/perception_bringup/launch/perception.launch.yaml` as its own
  `node:` (not loaded into `perception_container`), with `udp_bind_address: 10.8.0.1` and
  `udp_source_ip` from A2.
- Add `/camera_trigger` to the camera bag profiles in
  `src/perception/perception_bringup/config/bag_recorder/` (`all_sensors`,
  `all_sensors_compressed`, `camera_compressed`, `camera_only`).
- Foxglove: copy `src/perception/camera_trigger_msgs` into `docker/infrastructure.Dockerfile`.
- Topic healthchecker: add `/camera_trigger` to `topics:` in
  `src/interfacing/interfacing_bringup/config/interfacing.yaml`, and copy `camera_trigger_msgs` into
  `docker/interfacing.Dockerfile` (its generic subscription needs the message type).
- After #591 (fault collector) merges, optionally publish `wato_fault_msgs/Fault` from the node,
  at MODERATE or MILD only: level 2 or lower brakes the car.

## E. Hardware acceptance checklist

Preconditions: chrony synced (#560, done 2026-11-26), PPS polarity saved (A9), car outside with a GPS fix (A10).

- [ ] 20 Hz on `/camera_trigger` for 10 minutes, no gaps.
- [ ] `time_valid=true`, stamps on the 50 ms grid, residuals 0 or more (A11).
- [ ] Unplugging Ethernet gives ERROR within 1 s and recovers on replug.
- [ ] Trigger timing unchanged on the scope with reporting on.
- [ ] Measured `expected_latency` recorded and the default updated (A7).
- [ ] After section C: stamps identical across cameras for the same pulse, no frame-rate drop.

## F. Deferred from PR 1

| # | Feature | Spec | How to add it | Depends on |
|---|---|---|---|---|
| F1 | **USB serial fallback** | Base spec 5.1, 7.1; Ethernet delta 3 | A `SerialTransport` behind the `Transport` interface (`include/camera_trigger/transport.hpp`): termios raw 8N1, `poll()`, line assembly that discards up to the first newline after (re)connecting, reconnect. Add a `transport` parameter, a pty mode in the fake (`os.openpty()`), and `udev/99-camera-trigger.rules` (VID `0483`, PID `374b`, `ATTRS{serial}`). In Docker, `perception_bringup` needs `/dev:/dev` to see a replugged board: ask before changing the shared compose file. | The NUCLEO serial number: `udevadm info -a -n /dev/ttyACM0 \| grep serial`. Confirm the PID with `lsusb`. |
| F2 | **`STAT` line** (firmware health) | Ethernet delta 3.3 | Parse `STAT,1,<uptime_s>,<state>,<pps_err_ns>,<pps_glitches>,<skipped_pulses>,<stray_strobes>,<tx_drops>` in `protocol`, add the delta's WARN keys to the diagnostics, and make the fake send `STAT` once a second. | A1 (the confirmed format) |
| F3 | **Publishing reordered packets** | Ethernet delta 3.2 | In `SequenceTracker`, accept late packets up to 10 back (and un-count their gap). Add `--reorder-rate` to the fake. | Only worth it if `late_total` is ever non-zero on the car. |
| F4 | **Automated end-to-end tests** (partly done) | Base 5.5, Ethernet delta 6 | Done: `test/test_camera_trigger_node.cpp` runs the node in-process against a fake board on loopback (stamps, silence, gaps, duplicates, bad lines, PPS lost, bad parameters). Left: the same for `tools/fake_stm32.py` itself and its remaining fault options (reboot, clock offset, strobe misses), with `launch_testing` (`add_launch_test`) on a free port. | - |
| F5 | **Camera names in diagnostics** | Base 5.1 `camera_names` | A `camera_names` parameter mapping STB inputs to camera names. | A5 |

## How to pick this up

1. Read the package [README](../README.md) and run the fake board to work without hardware.
2. Close A items as the information arrives. Each says which file to edit. Protocol changes
   always go in together with the parser tests.
3. Start the next PRs from sections C and D, and pick F items as needed.
