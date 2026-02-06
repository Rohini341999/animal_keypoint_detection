from ultralytics import YOLO

# model = YOLO("/content/data/MyDrive/horse_pose_estimation/horse_26kp/dataset.yaml")  # build a new model from YAML
# model = YOLO("yolov8n-pose.pt")  # load a pretrained model (recommended for training)
model = YOLO(
    "/home/roha/Downloads/horse_26kp-20260206T005238Z-1-001/horse_26kp/yolov8s-pose.pt"
)
results = model.train(
    data="./dataset.yaml",
    epochs=100,
    imgsz=640,
    batch=-1,
    patience=30,
    name="yolov8s-pose-horse",
    project="./runs/pose",
    # Hyperparameters
)
