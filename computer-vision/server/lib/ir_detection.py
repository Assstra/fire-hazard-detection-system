import asyncio
import logging
import time
from typing import Any, AsyncGenerator, Dict, Optional
import cv2
import numpy as np
from flirpy.camera.lepton import Lepton
from lib import utils_frame_text
from lib.video_writer import VideoWriterService


logger = logging.getLogger(__name__)
TELEMETRY_H = 2


class InfraredDetectionService:
    def __init__(
        self,
        video_writer_svc: Optional[VideoWriterService] = None,
    ):
        self.video_writer_svc = video_writer_svc
        self.cam = Lepton()
        self.cam.setup_video()
        if not self.cam.cap.isOpened():
            raise RuntimeError("Camera could not be opened")
        self.cam_info = CameraInformation(
            width=int(self.cam.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            height=int(self.cam.cap.get(cv2.CAP_PROP_FRAME_HEIGHT) - TELEMETRY_H),
        )

    def detect_from_video_stream(self) -> AsyncGenerator[Dict[str, Any], None]:
        """Generator for continuous video detection"""

        async def detection_generator():
            try:
                while True:
                    ret, frame = self.cam.cap.read()
                    if not ret:
                        logger.warning("Failed to read frame")
                        break

                    # Detect position in frame
                    timestamp_start = time.time()
                    # height + 2 (for telemetry) & width * 2
                    frame = frame.reshape(
                        (self.cam_info.height + TELEMETRY_H, self.cam_info.width * 2)
                    )

                    # Gray16
                    frame_16 = frame.view(np.uint16)
                    frame_16 = frame_16[:-2, :]  # remove telemetry

                    # Gray8 to visualize
                    norm_frame = cv2.normalize(frame_16, None, 0, 255, cv2.NORM_MINMAX)
                    norm_frame = norm_frame.astype(np.uint8)

                    # Add frame to [] (in COLORMAP_JET, COLORMAP_INFERNO or COLORMAP_HOT)
                    color_frame = cv2.applyColorMap(norm_frame, cv2.COLORMAP_JET)
                    # frames.append(
                    #     {"gray16": frame_16, "gray8": norm_frame, "color": color_frame}
                    # )
                    utils_frame_text(
                        color_frame,
                        f"Temperature: {np.mean(frame_16):.2f}°C",
                        (10, 20),
                    )
                    self.video_writer_svc.write_frame(color_frame)

                    # we wait for the remaining time, if it didn't take the full second
                    elapsed_time = time.time() - timestamp_start
                    logger.info(
                        f"Detection took {elapsed_time:.4f} seconds, waiting for remaining time"
                    )
                    if elapsed_time < 1:
                        await asyncio.sleep(1 - elapsed_time)

            finally:
                self.cam.close()

        return detection_generator()

    def add_bounding_boxes_to_frame(
        self, image: np.ndarray, boxes: list[tuple[float, float, float, float]]
    ) -> np.ndarray:
        for box in boxes:
            x1, y1, x2, y2 = map(int, box)
            cv2.rectangle(image, (x1, y1), (x2, y2), (255, 255, 255), 2)
        return image


class CameraInformation:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height


class Box:
    def __init__(self, x: float, y: float, size: int, temperature: float):
        self.x = x
        self.y = y
        self.size = size
        self.temperature = temperature

    def xyxy(self) -> tuple[float, float, float, float]:
        return (self.x, self.y, self.x + self.size, self.y + self.size)
