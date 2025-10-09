# Camera Bringup

## Computer specific

The cameras are plugged into a switch that is connected to the main computer.

Type `ipconfig` to see network interfaces.

The switch is on `enp7s0f1` (the port of SFP network card closer to the motherboard), where:

en = Ethernet
p7 = PCI bus 7
s0 = slot 0
f1 = function 1

Key things to check for:
- Default and max recieve buffer size of 10485760
- Maximum Transmission Unit (MTU) value is 9000 for the interface:

## Camera Specific

To find information about the connected cameras, type:

`spinview`

to open an interactive GUI.

Software Note: we use composable nodes to connect to the Spinnaker SDK [to prevent undefined behaviour](https://docs.ros.org/en/iron/p/spinnaker_camera_driver/)

## Networking
Dnsmasq assigns static IP's to the cameras based on their MAC address. You can see the assignments [here](../interfacing_bringup/README.md).

To set the cameras to automatically search for a DHCP server on boot, do the following:
1. Auto force IP of camera in order to be able to access its configurations
2. Go to `Blackfly BFLY...` -> `Transport Layer Control`
3. Make sure the following are checked:
[x] GEV Supported Option
[x] GEV Current IPConfiguration LLA
[x] GEV Current IPConfiguration DHCP
[ ] GEV Current IPConfiguration Persistent IP

What this does is tell the blackfly camera to first attempt to retrieve an IP from a DHCP server (which in our case is the server on our computer setup with dnsmasq), and then resort to Link-Local for fallback. Do not enable Persistent IP as it will take precedent over DHCP.

### Blackfly GigE (Teledyne FLIR BFLY-PGE-23S6C-C)

    These are global shutter RGB cameras with a max frame rate of 41 fps. Please note that PTP IEEE 1588 is not supported (https://docs.ros.org/en/rolling/p/spinnaker_camera_driver/#configuring-ptp-ieee-1588)

    The serial numbers are:
    - 17453304
    - 17453317
    - 18542606

    Assuming the humble-devel version of [flir_camera_driver](https://github.com/ros-drivers/flir_camera_driver) is installed, test a camera with:

    ```sh
    ros2 launch spinnaker_camera_driver driver_node.launch.py camera_type:=blackfly serial:="'17453317'"
    ```

#### Lens (Fujinon CF12.5HA-1)

To write:
- intrinsics calibration: tuning and fixing focus
- extrinsics calibration: URDF and mounting to vehicle
