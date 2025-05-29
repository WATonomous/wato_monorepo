# Tracking

## Download nuScenes dataset
Download the desired dataset's .tgz file from https://www.nuscenes.org/nuscenes#download into wato_monorepo/src/perception/tracking/tracking/dataset_tracking/nuscenes  
Run extract_nusc.sh (in dataset_tracking/nuscenes) with the .tgz file name as argument

## Running tracking nodes
In docker-compose.perception.yaml, set tracking service command to  
/bin/bash -c "ros2 launch tracking tracking.launch.py" or  
/bin/bash -c "ros2 launch tracking nuscenes.launch.py" to run a specific node  
  
Set it to  
/bin/bash -c "ros2 launch tracking multi.launch.py" to launch multiple nodes together  
Add arguments of form 'node_exec_name:=false' to exclude nodes as required  
For details on ros parameters, see perception/tracking/config/tracking_params.yaml

## Key files
* tracking/core/ab3dmot.py contains Kalman filter and association logic  
* tracking/tracker.py receives Detection3DArray and publishes TrackedObstacleList obtained using ab3dmot tracker  
* tracking/nuscenes_publisher.py converts nuscenes data into Detection3DArray for publishing and generates visualization of responses from tracker.py compared to ground truth boxes