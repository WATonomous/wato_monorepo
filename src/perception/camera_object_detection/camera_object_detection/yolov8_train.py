from ultralytics import YOLO

yolo = YOLO('yolov8m.pt')
yolo.train(data='/perception_datasets/roboflow/traffic_light_roboflow_v3/data.yaml', epochs=300, freeze=10)
valid_results = yolo.val()
print(valid_results)
