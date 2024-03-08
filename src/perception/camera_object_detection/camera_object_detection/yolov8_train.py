from ultralytics import YOLO

yolo = YOLO('yolov8m.pt')
yolo.train(data='/perception_datasets/roboflow/bigger_traffic_light_roboflow_v1/data.yaml', epochs=300, freeze=10)
valid_results = yolo.val()
print(valid_results)
