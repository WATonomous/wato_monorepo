Go to watod-config.sh and in the "ACTIVE_MODULES" field put "perception" and "infrastructure". In the "MODE_OF_OPERATION" field put "develop"
-> Ex: ACTIVE_MODULES="perception infrastructure"
-> Ex: MODE_OF_OPERATION="develop"

CD into cloned directory and input following command: ./watod build tracking

After tracking image is built input following command to start appropriate containers: ./watod up tracking foxglove data_stream
-> This is going to build the images for foxglove and data_stream and then start up the containers
-> data_stream container won't immediately start. It will start up once a connection has been established with foxglove through port forwarding

Using the docker extension enter the dev container for tracking
-> Or you can use the following command as well: ./watod -t tracking

CD into ament_ws (ROS2 Workspace in the dev container)
-> Path: home/bolty/ament_ws

Run following command in ament_ws directory (container): colcon build

Run following command in monorepo directory: ./watod run data_stream ros2 bag play ./nuscenes/NuScenes-v1.0-mini-scene-0061/NuScenes-v1.0-mini-scene-0061_0.mcap --loop

Foxglove Setup For Annotations:
Select an "Image Panel"
Select topic /CAM_FRONT/image_rect_compressed
Select Calibration /CAM_FRONT/camer_info
Select "Sync Annotatiots" to be on
Reveal the /CAM_FRONT/image_markers_annotations image annotations