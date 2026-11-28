# GPS-referenced PTP for camera synchronization

**Status: planned.** Replace the direct GPS PPS input as the STM32's synchronization source with
PTP from a GPS-referenced grandmaster. The STM32 must synchronize its clock over Ethernet and
schedule the common camera trigger grid from that clock. The goal is for camera capture times to
be comparable with the host and other sensors, within a measured end-to-end error budget.
The deployment target is the NUCLEO-F767ZI; the PTP connection is Ethernet.

The team has set a **2.5 ms maximum pulse timestamp error**: when a pulse is marked time-valid,
its published timestamp must be within 2.5 ms of the physical trigger edge in the GPS-referenced
time domain. This bound applies to every pulse and includes synchronization, clock conversion and
output timestamping errors. At 20 Hz the trigger period is 50 ms; 5 ms corresponds to 200 Hz.
The 2.5 ms bound does not establish a camera-to-camera exposure-skew limit, which is still
undefined. The earlier 1 µs board-output skew criterion and PPS-specific timing criteria are
separate checks, not substitutes for the end-to-end timestamp requirement.

```text
GPS-referenced PTP grandmaster -- Ethernet / PTP --> STM32 synchronized clock
                 |                                      |
                 +-- Ethernet / PTP --> host clock      +--> timer --> camera trigger outputs
                                                        +--> pulse reports --> host --> frame stamps
```

The PwrPak7 supports PTP according to [NovAtel](https://docs.novatel.com/OEM7/Content/Configuration/PTP_Configuration.htm);
the installed receiver's firmware version, configuration and network path still need verification.
See the [implementation plan](../../../../docs/superpowers/plans/2026-11-26-camera-ptp.md)
for the proposed profile, acquisition path and acceptance gates. PPS is not the target
synchronization input to the STM32. A reference signal may still be useful for independent
measurement during validation.

## Existing implementation

The merged firmware builds for the NUCLEO-F767ZI and NUCLEO-H753ZI. Today TIM2 captures PPS
edges, estimates the timer rate and aligns a 20 Hz trigger grid to PPS. It holds the last rate on
PPS loss and reports `pps_locked=0`; it has no PTP synchronization implementation. Its Ethernet
code currently handles the trigger reporting path, ARP and ping, not PTP. See the
[firmware README](../README.md) and [scheduler](../app/include/cs/cs_sched.h).

The host node currently interprets the version 1 report's `pps_count`, `pulse_idx` and
`pps_locked` as a PPS-based grid and reconstructs timestamps using the host clock. Its
`time_valid` check does not verify PTP lock or GPS time quality. Camera frame association and car
integration remain open; see the [host open items](../../../perception/camera_trigger/docs/open_items.md).

## Work required

1. **Establish the reference.** Confirm the grandmaster, GPS time-validity indication, PTP
   version/profile, transport, domain, network topology and time scale. Determine the accuracy
   and timestamping support of both STM32 board variants, the host and intervening network
   equipment. Define clock ownership on the host so time synchronization services do not conflict.
2. **Synchronize the STM32 clock.** Implement a PTP client and clock discipline appropriate to the
   chosen hardware and profile, including packet timestamping, offset and path-delay estimation,
   frequency/phase correction, reference loss, clock steps and recovery. Verify the mapping between
   the disciplined clock and TIM2: synchronizing an Ethernet timestamp clock alone does not
   synchronize the timer used for GPIO trigger outputs.
3. **Schedule triggers from synchronized time.** Generate the 20 Hz grid from the synchronized
   time reference, maintaining deterministic output timing. Define behavior at acquisition,
   corrections, timer wrap, late deadlines and loss of synchronization, including whether pulses
   continue in holdover and when their timing must be marked invalid.
4. **Update the report contract and host together.** Define a versioned representation for pulse
   time, time scale, synchronization state and timing quality, and account for boot, packet loss,
   reordering and wrap. Update both firmware and host parsers, tests, diagnostics and message
   semantics together. Do not reinterpret version 1's PPS fields as PTP lock; existing consumers
   depend on their current meaning. Specify UTC/ROS conversion and leap-second handling explicitly.
5. **Associate camera frames with pulses.** Update the camera driver to match each frame to its
   trigger, handle dropped or ambiguous frames and propagate invalid timing. Account for camera
   trigger-to-exposure delay and variation; simultaneous outputs alone do not prove simultaneous
   exposures. Cameras can use the common hardware trigger even if they do not support PTP.

## Validation and acceptance

Apply the 2.5 ms pulse timestamp error limit to every valid pulse; define numeric limits for
camera-to-camera exposure skew, drift in holdover, acquisition and recovery time before final
hardware acceptance. Measure clock
offsets and trigger/exposure timing against an independent reference on the actual boards, network
and cameras, with a valid GPS fix. Include trigger output latency, camera exposure delay, report
and frame association, and time-scale conversion in the error budget.

Test startup, loss and recovery of GPS and PTP, grandmaster changes, network delay or disconnection,
clock corrections, STM32 reboot, timer wrap, delayed or lost reports and dropped frames. Expose
reference quality, synchronization state, offset, synchronization age and association failures in
diagnostics; affected timestamps must be invalid whenever the agreed bounds cannot be established.
Record the setup, measurements and pass/fail results. Current PPS simulation results are not PTP
or hardware acceptance evidence.
