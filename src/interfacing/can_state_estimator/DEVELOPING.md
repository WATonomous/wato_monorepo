# Developing can_state_estimator

## Design Rationale

CAN frames are read directly via SocketCAN rather than subscribing to OSCC topics because:
1. Eliminates the dependency on `oscc_interfacing` for feedback data
2. Lower latency — no extra ROS hop between CAN and state estimation
3. A single node handles both steering and wheel speed, keeping the state consistent

The wheelbase is looked up from TF (published by `robot_state_publisher` from `eve_description`) instead of being hardcoded. This keeps vehicle geometry defined in one place (the URDF) and automatically adapts if the model changes.

## Architecture

1. Constructor declares parameters only (lifecycle pattern)
2. On configure: opens a SocketCAN socket with a kernel-level filter for CAN IDs 0x2B0 and 0x4B0
3. On activate: starts a background thread that blocks on `read()` for CAN frames
4. Each CAN frame is decoded and the corresponding state is updated under a mutex
5. On each wheel speed frame: publishes velocity and integrates odometry
6. On each steering frame: publishes steering angle

The CAN read thread runs independently of the ROS executor. Publishers check `is_activated()` before publishing so the thread can safely run during transitions.

## CAN Frame Decoding

**Steering angle (0x2B0):**
- Bytes 0-1: int16 little-endian, scaled by 0.1 deg/bit
- Negated, divided by `steering_conversion_factor`, converted to radians

**Wheel speeds (0x4B0):**
- Four 12-bit values at byte offsets 0, 2, 4, 6
- Upper nibble masked: `raw = ((data[offset+1] & 0x0F) << 8) | data[offset]`
- Converted to km/h: `(int)(raw / 3.2) / 10.0`
- Byte layout: NW (left-front), NE (right-front), SW (left-rear), SE (right-rear)

## Thread Safety

The CAN read thread writes steering angle and wheel speeds. The odometry computation reads them. A single mutex protects all shared state. The lock is held briefly (just copying doubles), so contention is negligible.

Odometry state (x, y, theta, last_time) is only accessed from the CAN read thread so it does not need the mutex.

## Adding New CAN Signals

1. Add the CAN ID constant and add it to the kernel filter array in `on_configure`
2. Add a `process_*_frame()` method with the decoding logic
3. Add the dispatch case in `read_loop()`
4. Add any new publishers as lifecycle publishers and activate/deactivate them in the lifecycle callbacks
