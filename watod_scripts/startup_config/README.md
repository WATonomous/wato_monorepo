# CAN Interface Startup Setup

This service initializes the Kvaser CAN interface (500k bitrate) automatically on system boot.

## Installation

Run the following commands on the car's computer:

```bash
# 1. Install search file
sudo cp watod_scripts/startup_config/can-interface.service /etc/systemd/system/

# 2. Enable and start service
sudo systemctl daemon-reload
sudo systemctl enable --now can-interface.service
```

## Verification

```bash
systemctl status can-interface.service
ip link show can0
```
