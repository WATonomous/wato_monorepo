from ultralytics import YOLO

project='/perception_models' # dir to store trained model https://github.com/ultralytics/ultralytics/issues/3527

yolo = YOLO('yolov8m.pt')
yolo.train(data='/perception_datasets/roboflow/traffic_signs_roboflow_v0/data.yaml', epochs=300, freeze=10, project=project)
valid_results = yolo.val()
print(valid_results)