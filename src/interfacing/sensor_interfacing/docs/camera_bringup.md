# Camera Bringup

To publish raw frames from all Hikrobot cameras, run:

```sh
ros2 launch sensor_interfacing all_cameras_composed.yaml
```

> NOTE: Because ROS2 does serialization/deserialization regardless of middleware (even with Zenoh SHM), using 12 cameras at once is not possible without having them inside the same component container alongside any nodes that require camera data. As a result, cameras are launched inside the perception docker container alongside any perception nodes. This makes it so that all nodes that require camera do not do any serialization/deserialization to get images.

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
/opt/MVS/bin/MVS
```

We use [camera_aravis2](https://github.com/FraunhoferIOSB/camera_aravis2) to communicate with Hikrobot cameras throught the GenICam standard interface.

The config file located at sensor_interfacing/config/MV-CU013-80GC_master_config.mfs can be loaded in the MVS software to ensure camera settings are consistent.
