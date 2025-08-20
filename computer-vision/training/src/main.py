import os
import ultralytics
from ultralytics import settings, YOLO

ultralytics.checks()

settings.update(
    {"datasets_dir": "./data", "weights_dir": "./weights", "runs_dir": "./runs"}
)
print(settings)

model = YOLO("yolo11n.pt")
epochs = os.getenv("EPOCHS", 10)
results = model.train(data="./config/dfire.yml", epochs=int(epochs), imgsz=640)

model.export(format="engine")
model.export(format="onnx")
