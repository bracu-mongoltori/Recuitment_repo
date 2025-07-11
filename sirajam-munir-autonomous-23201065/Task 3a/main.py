from ultralytics import YOLO

# Load a model
model = YOLO("yolov8n.pt")  # build a pretrained model from scratch
results = model.train(data="data.yaml", epochs=50)  # train the model 
