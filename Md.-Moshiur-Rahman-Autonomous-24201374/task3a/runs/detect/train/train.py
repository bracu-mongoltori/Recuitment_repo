from ultralytics import YOLO
from IPython.display import Image, display


model = YOLO("yolov8s.pt")


results = model.train(
    data="dataset/bottle/data.yaml",
    epochs=50,
    imgsz=640,
    batch=16
)


