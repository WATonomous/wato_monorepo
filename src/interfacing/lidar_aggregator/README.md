# Lidar Aggregator
Package to merge multiple lidar pointclouds together into a single pointcloud.

## using Lidar Aggregator on ROS bags
```
./watod -m interfacing:dev up -d
./watod bag play <bag_name> --clock
## in another terminal, shell in for extra nodes:
./watod -t interfacing_bringup
ros2 launch lidar_aggregator lidar_runtime_fusion.yaml use_sim_time:=true
```

