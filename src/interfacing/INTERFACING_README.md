# Interfacing Packages

Directory of packages that have to do with interfacing. Information about each package can be found in their respective directory.

To see general network bringup information, see [the inferfacing bringup readme](./interfacing_bringup/README.md)

## Connecting to the Onboard Computer
The Eve onboard computer can either be accessed directly by connecting a desktop monitor and keyboard to it, or via tailscale.

The password to the user, as well as the tailscale login, is given in the WATonomous 1Password (accessible by leads).

```bash
ssh watonomous@$EVE_IP_ADDRESS
```

## Sensor Bringup Documentation
For documentation on bringing up each sensor (specifically the settings we have to set in each sensor's SDK). Refer to the following:

[**Camera Bringup**](./sensor_interfacing/docs/camera_bringup.md)
[**Lidar Bringup**](./sensor_interfacing/docs/lidar_bringup.md)
[**GPS Bringup**](./sensor_interfacing/docs/gps_bringup.md)

## Sensor Placement and Naming Convention
Eve's sensor placement follows cardinal directions as shown below. They are also mapped to specific indicies.

![alt text](img/image.png)

This naming convention is used for all sensor setup and transforms.

## Sub-System Diagram

## Custom messages

Custom messages may be used to help send messages between nodes. [see docs](../wato_msgs/MSGS_README.md)
