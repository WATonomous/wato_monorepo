from ultralytics import YOLO

#project='' # dir to store trained model https://github.com/ultralytics/ultralytics/issues/3527

yolo = YOLO('yolov8m.pt')

yolo.train(data='/perception_datasets/roboflow/traffic_signs_roboflow_v1/data.yaml', model='/home/bolty/ament_ws/src/camera_object_detection/runs/detect/train/weights/best.pt', epochs=500, weight_decay=0.001, freeze=10, resume=True)
valid_results = yolo.val()
print(valid_results)