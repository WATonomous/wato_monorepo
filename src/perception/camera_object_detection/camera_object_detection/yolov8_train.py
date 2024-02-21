from ultralytics import YOLO

yolo = YOLO('yolov8s.pt')
yolo.train(data='/perception_datasets/roboflow/traffic_light_roboflow_v0/data.yaml', epochs=300)
valid_results = yolo.val()
print(valid_results)
