# Developing camera_trigger

## Layout

The node is a thin ROS wrapper around small, ROS-free parts that are each unit-tested on their own.

| Part | Files | Does |
|---|---|---|
| Protocol | `include/camera_trigger/protocol.hpp`, `src/protocol.cpp` | Parses one report line (`TRG`, `BOOT`): checksum, framing, version, field checks |
| Sequence tracker | `sequence_tracker.{hpp,cpp}` | Next / gap / duplicate / late / board reboot, from the pulse `seq` |
| Pulse timing | `pulse_timing.{hpp,cpp}` | Receive time + report → pulse time on the PPS grid, `time_valid` |
| Transport | `transport.{hpp,cpp}` | `Transport` interface and `UdpTransport`: one datagram per line, kernel receive timestamps, source filter |
| Node | `camera_trigger_node.{hpp,cpp}` | Parameters, I/O thread, `/camera_trigger`, `/diagnostics`, logs |
| Fake board | `tools/fake_stm32.py` | Sends realistic reports, with fault injection |
| Node test | `test/test_camera_trigger_node.cpp` | Runs the node against a fake board on loopback |

Data flow for each datagram: `Transport::read` → `parseLine` → `SequenceTracker::update` →
`PulseTimer::compute` → publish.

`camera_trigger_core` (static, no ROS) holds the first four parts. `camera_trigger_component` is the
node, registered as the component `camera_trigger::CameraTriggerNode` with its own executable,
`camera_trigger_node`.

## Build and test

```bash
colcon build --packages-up-to camera_trigger
colcon test --packages-select camera_trigger && colcon test-result --verbose

# one test file
colcon test --packages-select camera_trigger --ctest-args -R test_pulse_timing
```

In CI this runs through `./watod test` for the perception module. Tests use Catch2 through
`wato_test` (`add_wato_test`).

| Test | Covers |
|---|---|
| `test_protocol` | Valid lines, every error category, limits, truncated lines and garbage |
| `test_sequence_tracker` | Gaps, duplicates, late packets, reboot threshold, wrap at 2^32 |
| `test_pulse_timing` | Grid snapping at every pulse index, second boundaries, jitter, clock errors (+-40 ms, +-0.2 s, +-2 s), second-offset rule, unlocked pulses |
| `test_transport` | UDP on loopback: kernel timestamps, oversize datagrams, source filter, busy port |
| `test_camera_trigger_node` | The node end to end: a fake board in the test sends reports on loopback. Exact stamps on the PPS grid, ERROR on silence, gaps / duplicates / bad lines counted and not published, PPS-lost warning, bad parameters rejected |

`test_camera_trigger_node` waits on conditions instead of sleeping for fixed times, and keeps
sending until the node reports a healthy state, so a busy CI machine doesn't make it fail. The
remaining fault options of `tools/fake_stm32.py` (see the README's table) are still checked by hand.

## Implementation notes

- **Threads.** Reports are read on a dedicated I/O thread: `poll()` with a 100 ms timeout, so
  shutdown is quick and nothing spins. The parser, sequence tracker and pulse timer are only used
  on that thread. Everything the diagnostics read is in `Stats`, guarded by one mutex. The
  diagnostics timer runs on the executor.
- **Clocks.** `t_rx` and stamps use the system clock (`CLOCK_REALTIME`, kernel receive timestamp
  when available). Every rate, timeout and window uses `steady_clock`, so a system-clock step (for
  example chrony's first correction) can't fake a stale or rate error.
- **Bad configuration** throws from the constructor, so the node fails at startup with a clear
  message instead of retrying forever.
- **Adding a transport** (serial fallback, open item F1): implement `Transport` and pick it with a
  parameter. The parser, timing and node logic don't change. A serial transport must split the
  byte stream into lines and discard up to the first newline after (re)connecting.
- **Changing the protocol:** update [docs/serial_protocol.md](docs/serial_protocol.md),
  `src/protocol.cpp` and `test/test_protocol.cpp` together, and tell the firmware owner.

## Testing on WSL2

Seen on a WSL2 development machine, not on Linux hosts or the car:

- The system clock gets stepped by about a second every so often to resync with Windows. The node
  then reports a brief silence or one bad-residual warning, which is correct behaviour for a
  clock step.
- ROS discovery between processes can fail (no multicast), and loopback drops UDP datagrams of
  1500 bytes or more. A Fast DDS profile with unicast discovery on 127.0.0.1 and
  `maxMessageSize` 1400 (`FASTRTPS_DEFAULT_PROFILES_FILE`) works around both.
