import asyncio
import logging
import time
import cv2
from fastapi import HTTPException
import numpy as np
from ultralytics import YOLO
from typing import AsyncGenerator, Dict, Any
from lib import DetectionKind, Position

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
            logger.info(
                f"Successfully loaded model from {self.model_path} with confidence {self.confidence_threshold}"
            )
        except Exception as e:
            logger.error(f"Failed to load model: {e}")
            raise HTTPException(status_code=500, detail=f"Failed to load model: {e}")

    def detect_position_from_image(self, image: np.ndarray) -> Position:
        """Detect the overall position of the boxes in the image"""
        if self.model is None:
            raise HTTPException(status_code=500, detail="Model not loaded")

        try:
            results = self.model.predict(
                image, stream=True, conf=self.confidence_threshold
            )
            fire_position = Position.NONE
            # smoke_position = Position.NONE
            center_x = image.shape[1] // 2

            for result in results:
                boxes = result.boxes
                if boxes is None:
                    continue

                fire_box_centers = []
                # we ignore smoke detection for now
                # smoke_box_centers = []
                for box in boxes:
                    conf = box.conf.cpu().numpy()[0]
                    if conf < self.confidence_threshold:
                        continue
                    cls = int(box.cls.cpu().numpy()[0])
                    x1, _, x2, _ = box.xyxy.cpu().numpy()[0]
                    box_center_x = (x1 + x2) // 2
                    class_name = get_class_name(cls, self.model)
                    match class_name:
                        case "fire":
                            fire_box_centers.append(box_center_x)
                        case "smoke":
                            pass
                        case _:
                            raise Exception(
                                f"Unknown class detected: {class_name}, id: {cls}"
                            )

            if fire_box_centers:
                fire_position = calculate_position(fire_box_centers, center_x)
            # we ignore smoke position for now
            # if smoke_box_centers:
            #     smoke_position = calculate_position(smoke_box_centers, center_x)

            return fire_position

        except Exception as e:
            logger.error(f"Detection error: {e}")
            return Position.NONE

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

                    # Detect position in frame
                    timestamp_start = time.time()
                    fire_position = self.detect_position_from_image(frame)

                    if fire_position != Position.NONE:
                        event_data = {
                            "type": DetectionKind.RGB,
                            "position": fire_position,
                            "timestamp_start": timestamp_start,
                            "frame_info": {
                                "width": frame.shape[1],
                                "height": frame.shape[0],
                                "channels": frame.shape[2],
                            },
                        }
                        yield event_data

                    # we wait for the remaining time, if it didn't take the full second
                    elapsed_time = time.time() - timestamp_start
                    logger.info(
                        f"Detection took {elapsed_time:.4f} seconds, waiting for remaining time"
                    )
                    if elapsed_time < 1:
                        await asyncio.sleep(1 - elapsed_time)

            finally:
                cap.release()

        return detection_generator()


def get_class_name(cls: int, model: YOLO) -> str | None:
    """Get class name from class index"""
    if cls < len(model.names):
        return model.names[cls]
    else:
        return None


def calculate_position(box_centers: list[np.float32], center_x: int) -> Position:
    """Determine overall position of `boxes_centers` from the image center `center_x`"""
    logger.info(f"Calculating intersection for centers: {box_centers}")
    intersection_x = sum(box_centers) / len(box_centers)
    logger.info(f"Intersection X: {intersection_x}, Center X: {center_x}")

    if abs(intersection_x - center_x) < 75:  # Tolerance for center
        return Position.CENTER
    elif intersection_x < center_x:
        return Position.LEFT
    else:
        return Position.RIGHT
