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

## NITROS lifecycle fork compatibility

The lifecycle fork pinned in `docker/perception.Dockerfile` changes the NITROS
C++ binary interface, including format callbacks from `rclcpp::Node&` to
`NitrosNodeInterfaces`. Its core library cannot be combined with NVIDIA's
prebuilt NITROS type libraries or image processing nodes, even when both report
version 4.5.0. The Dockerfile therefore also includes source builds of the image,
camera info, tensor list, and compressed image types, plus the rectifier and
H.264 encoder, against the same fork.

If the fork's core library ends up loading NVIDIA's prebuilt type libraries
instead of the fork-built ones, the component container crashes right after
`Successfully allocated 3 CUDA streams`, and lifecycle service timeouts follow.
The timeouts are a side effect of the crash: the fork is
calling NVIDIA's packaged libraries in `/opt/ros/jazzy/lib` instead of its own.

To prevent this, the image does not install NVIDIA's apt packages for these
components (`ros-jazzy-isaac-ros-nitros`, `-image-proc`, `-h264-encoder`,
`-cvcuda-utils`), only their binary dependencies. Do not add them back, directly
or through a package that depends on them; doing so puts the incompatible
libraries in `/opt/ros/jazzy`.

These packages are built with `BUILD_TESTING=OFF` and are not covered by
`watod test perception`. When changing the pin, check lifecycle behavior,
rectification, and H.264 encoding on a GPU machine.

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
