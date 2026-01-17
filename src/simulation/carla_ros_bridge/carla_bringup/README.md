# carla_bringup

Launch files and configuration for the CARLA ROS 2 bridge.

## Overview

This package provides the main entry point for launching the CARLA ROS 2 bridge. It contains launch files that coordinate all bridge nodes and a centralized configuration file.

## Usage

```bash
ros2 launch carla_bringup carla_bridge.launch.yaml
```

### Launch Arguments

See available arguments with:

```bash
ros2 launch carla_bringup carla_bridge.launch.yaml --show-args
```

## Configuration

All node parameters are configured in `config/carla_bridge.yaml`. Each node section uses the ROS 2 parameter format with `ros__parameters`.

## Package Structure

```
carla_bringup/
├── config/
│   └── carla_bridge.yaml    # Centralized configuration
├── launch/
│   └── carla_bridge.launch.yaml
└── package.xml
```
