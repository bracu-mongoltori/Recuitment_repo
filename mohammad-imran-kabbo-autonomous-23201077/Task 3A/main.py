# Import the YOLO class from the ultralytics package
from ultralytics import YOLO

# Load a pretrained YOLOv8n model (nano version) for transfer learning
model = YOLO('yolov8n.pt')

# Train the model on your custom dataset
# data       → path to your data.yaml file (defines train/test image paths and class names)
# epochs     → number of training iterations (here, 50 full passes over the dataset)
# imgsz      → resize all input images to 640x640 pixels before feeding into the model
results = model.train(
    data='E:/Task 3A/data.yaml',
    epochs=50,
    imgsz=640
)
