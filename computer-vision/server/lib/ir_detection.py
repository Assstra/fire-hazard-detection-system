import asyncio
import logging
import time
from typing import Any, AsyncGenerator, Dict, Optional

import cv2
import numpy as np
from flirpy.camera.lepton import Lepton

from lib import DetectionKind, Position, utils_frame_text
from lib.video_streaming import VideoStreamingService
from lib.video_writer import VideoWriterService

logger = logging.getLogger(__name__)
TELEMETRY_H = 2


class Box:
    def __init__(self, x1: int, y1: int, x2: int, y2: int, temperature: float):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.temperature = temperature

    def xyxy(self) -> tuple[int, int, int, int]:
        return (self.x1, self.y1, self.x2, self.y2)


class InfraredDetectionService:
    def __init__(
        self,
        raw_video_writer_svc: Optional[VideoWriterService] = None,
        video_writer_svc: Optional[VideoWriterService] = None,
        video_streaming_svc: Optional[VideoStreamingService] = None,
    ):
        self.raw_video_writer_svc = raw_video_writer_svc
        self.video_writer_svc = video_writer_svc
        self.video_streaming_svc = video_streaming_svc
        self.cam = Lepton()
        self.cam.setup_video()
        if not self.cam.cap.isOpened():
            raise RuntimeError("Camera could not be opened")
        self.cam_info = CameraInformation(
            width=int(self.cam.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            height=int(self.cam.cap.get(cv2.CAP_PROP_FRAME_HEIGHT) - TELEMETRY_H),
        )
        logger.info(f"height: {self.cam_info.height}, width: {self.cam_info.width}")
        # load first image
        time.sleep(1)  # wait for camera to initialize
        ret, frame = self.cam.cap.read()
        if not ret:
            raise RuntimeError("Failed to read initial frame from camera")
        self.latest_frame = frame

    def close(self):
        self.cam.close()

    def detect_from_video_stream(self) -> AsyncGenerator[Dict[str, Any], None]:
        """Generator for continuous video detection"""

        async def detection_generator():
            try:
                while True:
                    ret, frame = self.cam.cap.read()
                    if not ret:
                        logger.warning("Failed to read frame")
                        break
                    if self.raw_video_writer_svc:
                        self.raw_video_writer_svc.write_frame(frame)

                    timestamp_start = time.time()

                    frame_16 = self.preprocess_into_frame16(frame)
                    detections, annotated_frame = self.detect_hotspots(
                        frame_16, min_area=50
                    )

                    # Add temperature info to display frame
                    avg_temp = np.mean(frame_16)
                    utils_frame_text(
                        annotated_frame,
                        f"Avg Temp: {avg_temp:.1f}°C | Hotspots: {len(detections)}",
                        (10, 20),
                        (255, 255, 255),
                    )

                    self.send_frame(annotated_frame)

                    if detections:
                        event_data = {
                            "type": DetectionKind.IR,
                            "position": Position.NONE,
                            "timestamp_start": timestamp_start,
                            "frame_info": {
                                "width": annotated_frame.shape[1],
                                "height": annotated_frame.shape[0],
                                "channels": annotated_frame.shape[2],
                            },
                            # "hotspots": [
                            #     {
                            #         "x": box.x,
                            #         "y": box.y,
                            #         "width": box.size,
                            #         "height": box.size,
                            #         "temperature": box.temperature,
                            #     }
                            #     for box in detections
                            # ],
                            # "hotspot_count": len(detections),
                            # "avg_temperature": float(np.mean(frame_16)),
                        }
                        yield event_data

                    # we wait for the remaining time, if it didn't take the full second
                    elapsed_time = time.time() - timestamp_start
                    logger.info(
                        f"Detection took {elapsed_time:.4f} seconds, waiting for remaining time"
                    )
                    if elapsed_time < 0.1:
                        await asyncio.sleep(0.1 - elapsed_time)

            except asyncio.CancelledError:
                logger.debug("IR detection generator cancelled")
                raise
            except Exception as e:
                logger.error(f"Error in IR detection generator: {e}")
            finally:
                logger.info("Closing IR camera")
                self.cam.close()

        return detection_generator()

    def detect_hotspots(
        self, frame_16: np.ndarray, min_area: int = 50, temp_threshold: float = 75.0
    ) -> tuple[list[Box], np.ndarray]:
        # see notebook -/computer-vision/training/notebooks/camera.ipynb
        temp_frame = (frame_16.astype(np.uint16) / 100.0) - 273.15

        # create a binary mask for temperatures above threshold
        hot_mask = (temp_frame > temp_threshold).astype(np.uint8) * 255

        # apply morphological operations to clean up the mask
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        hot_mask = cv2.morphologyEx(hot_mask, cv2.MORPH_CLOSE, kernel)
        hot_mask = cv2.morphologyEx(hot_mask, cv2.MORPH_OPEN, kernel)

        # find contours of hot areas
        contours, _ = cv2.findContours(
            hot_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        bounding_boxes = []

        # create an annotated frame from the 8-bit version for visualization
        annotated_frame = cv2.normalize(frame_16, None, 0, 255, cv2.NORM_MINMAX).astype(
            np.uint8
        )
        annotated_frame = cv2.applyColorMap(annotated_frame, cv2.COLORMAP_JET)
        annotated_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
        annotated_frame = cv2.resize(annotated_frame, (640, 480))

        for contour in contours:
            # Filter by minimum area
            area = cv2.contourArea(contour)
            if area < min_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            # Calculate average temperature in the bounding box region
            roi_temp = temp_frame[y : y + h, x : x + w]
            avg_temperature = float(np.mean(roi_temp[roi_temp > temp_threshold]))

            bounding_boxes.append(Box(x, y, x + w, y + h, avg_temperature))

            cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.putText(
                annotated_frame,
                f"{avg_temperature:.2f}°C",
                (x, y - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                1,
            )

            logger.info(
                f"Hotspot detected: {avg_temperature:.1f}°C at ({x}, {y}) with area {area}"
            )

        return bounding_boxes, annotated_frame

    def preprocess_into_frame16(self, frame: np.ndarray) -> np.ndarray:
        # height + 2 (for telemetry) & width * 2
        frame = frame.reshape(
            (self.cam_info.height + TELEMETRY_H, self.cam_info.width * 2)
        )
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        # Gray16
        frame_16 = frame.view(np.uint16)
        frame_16 = frame_16[:-TELEMETRY_H, :]  # remove telemetry
        return frame_16

    def send_frame(self, frame: np.ndarray):
        if self.video_writer_svc:
            self.video_writer_svc.write_frame(frame)
        if self.video_streaming_svc:
            self.video_streaming_svc.write_frame(frame)


class CameraInformation:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
