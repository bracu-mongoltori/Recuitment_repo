from ultralytics import YOLO

# Load the trained YOLOv8 model (use your custom trained weights)
model = YOLO('runs/detect/train/weights/best.pt') 

# Run prediction on a single image
results = model.predict(
    source='E:/Task 3A/dataset/Plastic Bottle Image Dataset/test\images/-2_jpg.rf.56c6648de860249fe106b20ce690a650.jpg',   # path to the input image
    save=True,                         # save the result with bounding boxes
    project='predicted_photos',        # parent folder to save results
    name='result',                     # subfolder inside project
    save_txt=True                    # don't save coordinates to .txt
)

print("✅ Image prediction done! Saved in predicted_photos/result/")
