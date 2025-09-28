# Interfacing Bringup

Bringup package used to launch drivers for Eve.

## Network Configuration
Given that some of our sensors are connected via ethernet, some extra work needs to be done to ensure that all sensors are properly configured and connectable on startup.

Doing so requires knowledge in networking, which can get pretty convoluted.

At a high level, here's what needs to happen in order to make this work:
- We configure the SFP port to act as a local subnet with a DHCP server. 
  - That way any sensor asking for an IP address on startup will get it from our local PC's DHCP server instead of the internet.
  - This means that when the car is NOT connected to the internet, sensors can still be configured properly.
  - Without this, sensors will be unable to acquire an IP address, or alternatively cycle between different IP addresses periodically. Both are not ideal.
- We ensure that our ethernet port on the motherboard has a direct pathway to the internet.
  - We still want to be able to connect the car to the internet so that we can SSH into it for development.
  - We want to do so without exposing our sensors to the internet without a 

## Current Drivers

### Novatel GPS Driver
**Link to GitHub Repo:** https://github.com/novatel/novatel_oem7_driver/tree/humble
**Usage Pattern:** This driver is available as a rosdep key, usage of this driver consists of depending on the released version in `package.xml` and launching.
