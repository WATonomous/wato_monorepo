# Interfacing Bringup

Bringup package used to launch drivers for Eve.

## Network Configuration
Given that some of our sensors are connected via ethernet, some extra work needs to be done to ensure that all sensors are properly configured and connectable on startup.

Doing so requires knowledge in networking, which can get pretty convoluted.

At a high level, here's what needs to happen in order to make this work:
- We configure the SFP port to act as a local subnet with a DHCP server. (using common IP range for local network 10.x.x.x/16) 
  - That way any sensor asking for an IP address on startup will get it from our local PC's DHCP server instead of the internet.
  - This means that when the car is NOT connected to the internet, sensors can still be configured properly.
  - Without this, sensors will be unable to acquire an IP address, or alternatively cycle between different IP addresses periodically. Both are not ideal.
- We ensure that our ethernet port on the motherboard has a direct pathway to the internet.
  - We still want to be able to connect the car to the internet so that we can SSH into it for development.
  - We want to do so without exposing our sensors to the internet without a 

### Installing dnsmasq
To setup a computer to act as a DHCP server, we will be installing `dnsmasq` on the robot PC.

```bash
sudo apt update
sudo apt install dnsmasq -y
```

We will be doing static IP assignments by MAC address. Each device has a unique MAC address giving us a way to determine its identity.

Edit `/etc/dnsmasq.d/robot-net.conf`
```conf
interface=enp7s0f1
bind-interfaces

# General DHCP pool (fallback)
dhcp-range=10.8.0.100,10.8.0.200,12h

# Static assignments by MAC
dhcp-host=00:B0:9D:1A:F0:0E,10.8.0.18   # Blackfly Cam 1
dhcp-host=00:B0:9D:0A:50:F8,10.8.0.19   # Blackfly Cam 2
dhcp-host=00:B0:9D:0A:51:05,10.8.0.20   # Blackfly Cam 3
dhcp-host=00:21:66:04:A1:ED,10.8.0.8   # Novatel GPS/IMU
```

We do not setup a gateway. That is, we do not tell our sensors how to reach networks outside their subnet (ie. the internet)

### Configuring Netplan
Edited `/etc/netplan/01-netcfg.yaml`
```yaml
network:
  version: 2
  ethernets:
    enp6s0:        # Internet port
      dhcp4: true
    enp7s0f1:      # SFP port for robot network
      addresses:
        - 10.8.0.1/16
      mtu: 9000
```

### Clashes with Network Manager
If `ip a show enp7s0f1` shows an IP different from the one you configured with Netplan, then there is a chance that another tool is managing `enp7s0f1` and overriding Netplan. In our case, NetworkManager was overriding.

To see which profile controls `enp7s0f1`:
```bash
nmcli device status
nmcli connection show
```

If managed by profile "Wired connection 1", we modify NetworkManager's existing profile (or create a new one):
```bash
sudo nmcli connection modify "Wired connection 1" ipv4.method manual ipv4.addresses 10.8.0.1/16
sudo nmcli connection modify "Wired connection 1" 802-3-ethernet.mtu 9000
sudo nmcli connection down "Wired connection 1"
sudo nmcli connection up "Wired connection 1"
```

## Current Drivers

### Novatel GPS Driver
**Link to GitHub Repo:** https://github.com/novatel/novatel_oem7_driver/tree/humble
**Usage Pattern:** This driver is available as a rosdep key, usage of this driver consists of depending on the released version in `package.xml` and launching.

### Teledyne Blackfly GigE Driver
**Link to GitHub Repo:** https://github.com/ros-drivers/flir_camera_driver/tree/humble-devel
**Usage Pattern:** This driver is available as a rosdep key, usage of this driver consists of depending on the released version in `package.xml` and launching.

**More documentation on setup:** [Camera Bringup](../sensor_interfacing/camera_bringup.md)