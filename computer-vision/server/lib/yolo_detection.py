import asyncio
import logging
import time
import cv2
from fastapi import HTTPException
import numpy as np
from ultralytics import YOLO
from typing import AsyncGenerator, Dict, Any, List

logger = logging.getLogger(__name__)


class YOLODetectionService:
    """Service for fire/smoke detection using YOLOv11 model"""

    def __init__(self, model_path: str, confidence_threshold: float):
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.model = None
        self.load_model()

    def load_model(self):
        """Load YOLOv11 model"""
        try:
            self.model = YOLO(self.model_path, task="detect")
            logger.info(f"Successfully loaded model from {self.model_path} with confidence {self.confidence_threshold}")
        except Exception as e:
            logger.error(f"Failed to load model: {e}")
            raise HTTPException(status_code=500, detail=f"Failed to load model: {e}")

    def detect_from_image(self, image: np.ndarray) -> List[Dict[str, Any]]:
        """Detect in image"""
        if self.model is None:
            raise HTTPException(status_code=500, detail="Model not loaded")

        try:
            results = self.model.predict(image, stream=True, conf=self.confidence_threshold)
            detections = []

            for result in results:
                boxes = result.boxes
                if boxes is None:
                    continue
                for box in boxes:
                    # Extract detection info
                    conf = float(box.conf.cpu().numpy()[0])
                    cls = int(box.cls.cpu().numpy()[0])
                    xyxy = box.xyxy.cpu().numpy()[0]

                    # Get class name
                    class_name = (
                        self.model.names[cls]
                        if cls < len(self.model.names)
                        else "unknown"
                    )

                    detection = {
                        "class": class_name,
                        "confidence": conf,
                        "bbox": {
                            "x1": float(xyxy[0]),
                            "y1": float(xyxy[1]),
                            "x2": float(xyxy[2]),
                            "y2": float(xyxy[3]),
                        },
                        "timestamp": time.time(),
                    }
                    detections.append(detection)

            return detections

        except Exception as e:
            logger.error(f"Detection error: {e}")
            return []

    def detect_from_video_stream(
        self, video_source: int = 0
    ) -> AsyncGenerator[Dict[str, Any], None]:
        """Generator for continuous video detection"""

        async def detection_generator():
            cap = cv2.VideoCapture(video_source)

            if not cap.isOpened():
                logger.error(f"Cannot open video source: {video_source}")
                return

            try:
                while True:
                    ret, frame = cap.read()
                    if not ret:
                        logger.warning("Failed to read frame")
                        break

                    # Detect objects in frame
                    detections = self.detect_from_image(frame)

                    # Yield detection event if any detections found
                    if detections:
                        event_data = {
                            "type": "yolo_detection",
                            "detections": detections,
                            "frame_info": {
                                "width": frame.shape[1],
                                "height": frame.shape[0],
                                "channels": frame.shape[2],
                            },
                        }
                        yield event_data

                    # Small delay to prevent overwhelming the client
                    await asyncio.sleep(1)

            finally:
                cap.release()

        return detection_generator()
