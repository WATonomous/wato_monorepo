# Camera Bringup

The Hikrobot cameras are NITROS nodes and run inside the perception component
container. They are launched as part of the perception stack:

```sh
ros2 launch perception_bringup perception.launch.yaml
```

When testing not directly on the car (e.g. with a rosbag), disable the live cameras as follows:

```sh
ros2 launch perception_bringup perception.launch.yaml live_cameras:=false
```

To load the cameras into an existing component container on their own (e.g. for
debugging or bag recording), use the camera launch directly:

```sh
ros2 launch perception_bringup all_cameras_composed.yaml container_name:=<container>
```

> NOTE: Because ROS2 does serialization/deserialization regardless of middleware (even with Zenoh SHM), using 12 cameras at once is not possible without having them inside the same component container alongside any nodes that require camera data. As a result, cameras are launched inside the perception docker container alongside any perception nodes. This makes it so that all nodes that require camera do not do any serialization/deserialization to get images.

> NOTE: The camera pipeline runs on NVIDIA Isaac ROS (NITROS): frames are GPU-rectified (`isaac_ros_image_proc`) and the rectified stream is NVENC-encoded to H.264 on `image_rect_h264`, with an optional JPEG stream on `image_rect_compressed` (toggle via the `enable_jpeg` launch arg, which is CPU-bound).

## Computer specific

The cameras are plugged into a switch that is connected to the main computer.

Type `ipconfig` to see network interfaces.

The switch is on `enp8s0f1` (the port of SFP network card closer to the motherboard), where:

en = Ethernet
p8 = PCI bus 7
s0 = slot 0
f1 = function 1

Key things to check for:
- Default and max recieve buffer size of 10485760
- Maximum Transmission Unit (MTU) value is 9000 for the interface:

## Hikrobot Camera Specific

To configure permanent settings for cameras, use the MVS tool located at:

```sh
/opt/MVS/bin/MVS.sh
```

We use [camera_aravis2](https://github.com/FraunhoferIOSB/camera_aravis2) to communicate with Hikrobot cameras throught the GenICam standard interface.

The config file located at perception_bringup/config/MV-CU013-80GC_master_config.mfs can be loaded in the MVS software to ensure camera settings are consistent.

## Timestamps

The Hikrobot cameras do not support PTP. By default, each image is stamped with the time its
first packet reached the computer, which includes exposure, readout, network and driver delay
(varies per frame by tens of ms).

With `device_clock_stamp_enable: true` (in `config/hikrobot_camera_config.yaml`), the driver
instead stamps each image with the camera's internal capture timestamp, mapped onto the computer
clock per camera using the minimum of `arrival - device` over a sliding window:

| Parameter | Default | Meaning |
|---|---|---|
| `device_clock_stamp_enable` | `false` | Use the device clock. `false` keeps the arrival-time stamps |
| `device_clock_window_s` | `10.0` | Offset estimation window. Arrival time is used for the first window after start-up |
| `device_clock_latency_ms` | `0.0` | Constant transfer latency subtracted from stamps. Calibrate before changing |

When enabled, every camera logs a summary every 5 s:

```text
(camera_pano_nn) device clock: converged=1 offset=... s (change +0.012 ms) arrival-stamp min/mean/max=7.90/9.40/31.20 ms clamped=0 resets=0 frames=100
```

`change` should stay well under 1 ms, `arrival-stamp min` must be >= 0, and `clamped` and
`resets` should be 0. To measure stamp jitter for all cameras, run
`camera_aravis2/scripts/stamp_jitter.py` from the `camera_aravis2_nitros` repository.
