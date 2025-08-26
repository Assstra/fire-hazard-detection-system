import logging
import os
import sys

import ultralytics
from ultralytics import YOLO, settings

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

ultralytics.checks()

activate_mlflow = isinstance(os.getenv("ACTIVATE_MLFLOW"), str)
if activate_mlflow:
    # see https://github.com/ultralytics/ultralytics/blob/main/ultralytics/utils/callbacks/mlflow.py
    mlflow_experiment_name = os.getenv("MLFLOW_EXPERIMENT_NAME")
    mlflow_tracking_uri = os.getenv("MLFLOW_TRACKING_URI")
    if (
        (mlflow_experiment_name is None)
        or (mlflow_tracking_uri is None)
    ):
        logger.error(
            "You have activated MLFLOW, but one or more required environment variables are not set (ex: MLFLOW_EXPERIMENT_NAME, MLFLOW_TRACKING_URI)"
        )
        sys.exit(1)

settings.update(
    {
        "datasets_dir": "./data",
        "weights_dir": "./weights",
        "runs_dir": "./runs",
        "mlflow": activate_mlflow,
    }
)
logger.info("Settings updated: %s", settings)

base_model = os.getenv("BASE_MODEL", "yolo11m.pt")
model = YOLO(base_model)
epochs = int(os.getenv("EPOCHS", 10))
results = model.train(
    data="./config/dfire.yml", task="detect", epochs=epochs, imgsz=640
)

export_formats = os.getenv("EXPORT_FORMATS")
if export_formats is not None:
    for fmt in export_formats.split(","):
        model.export(format=fmt)

logger.info("Training and export completed.")
