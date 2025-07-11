from ultralytics import YOLO

# Load your trained model
model = YOLO("runs/detect/train/weights/best.pt")

# Run inference on your video
results = model.predict(
    source="E:/Task03_a/test_video/WIN_20250709_03_19_24_Pro.mp4",
    conf=0.2,
    save=True
)
