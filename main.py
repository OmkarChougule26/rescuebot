from ultralytics import YOLO

# This line will automatically download 'yolo11n.pt' if you don't have it
model = YOLO("yolo11n.pt") 
print("Model loaded successfully!")