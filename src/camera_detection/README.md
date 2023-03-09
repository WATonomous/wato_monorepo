# README.md

Building and running the node without Docker:
```
colcon build --packages-select common_msgs camera_detection
. install/setup.bash
ros2 run camera_detection camera_detection_node
```

The node expects the YOLOv5 model to be located in a `/perception_models` directory. If running through Docker, this is done for you in the image build. Otherwise if you are developing without Docker, you can run the commands here:
```
sudo mkdir -m 777 -p /perception_models
wget -P /perception_models https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt
```

By default, the node will subscribe to the topic `/camera/right/image_color` and publish annotated images to `/annotated_img` and an obstacle detection list to `/obstacles`.