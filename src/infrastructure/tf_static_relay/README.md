# tf_static_relay

**Temporary package** — remove once [ros2/rosbag2#2342](https://github.com/ros2/rosbag2/pull/2342) is released and available in our ROS distribution.

## Problem

When recording bags with `--max-bag-size` or `--max-bag-duration`, `/tf_static` messages are only captured in the first chunk. Static transforms use transient local QoS and are published once at startup, so subsequent bag chunks are missing them.

## Solution

This node subscribes to `/tf_static`, accumulates all static transforms, and republishes them at a fixed rate (default 1 Hz). This ensures every bag chunk contains the static transforms.

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `publish_rate` | `1.0` | Republish rate in Hz |
