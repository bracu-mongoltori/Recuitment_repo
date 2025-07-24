from ultralytics import YOLO

# Load the trained YOLOv8 model
model = YOLO('runs/detect/train/weights/best.pt')  # or 'yolov8n.pt'

# Run prediction on a video file
results = model.predict(
    source='E:/Task 3A/bottle_vid.mp4',   # path to the input video
    save=True,                         # save the result with bounding boxes
    project='predicted_videos',        # parent folder to save results
    name='result_video',               # subfolder inside project
    save_txt=True                   # don't save coordinates to .txt
)

print("✅ Video prediction done! Saved in predicted_videos/result_video/")
