from ultralytics import YOLO

# Load the YOLOv8 model
model = YOLO("models/yolo11n_8_up2.pt")

# Export the model to TensorFlow Lite format with int8 quantization
model.export(format="engine", int8=True, data="./models/google_colab_config.yaml")
