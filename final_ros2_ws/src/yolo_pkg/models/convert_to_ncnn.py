from ultralytics import YOLO

# Prompt user for the model path
model_path = input("Enter the path to your YOLO model (e.g., '/ros2/yolo/best.pt'): ")

# Load a YOLO11n PyTorch model
model = YOLO(model_path)

# Export the model to NCNN format
model.export(format="ncnn")  # creates 'yolo11n_ncnn_model'

