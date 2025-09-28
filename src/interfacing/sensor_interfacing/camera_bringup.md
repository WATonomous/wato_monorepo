# Camera Bringup

## Computer specific 

The cameras are plugged into a switch that is connected to the main computer.

Type `ipconfig` to see network interfaces.

The switch is either on `enp7s0f0` or `enp7s0f1`, where:

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

### Blackfly GigE (Teledyne FLIR BFLY-PGE-23S6C-C)

    These are global shutter RGB cameras with a max frame rate of 41 fps.

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
