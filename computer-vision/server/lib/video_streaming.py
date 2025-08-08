import asyncio
import logging
from typing import AsyncGenerator
import cv2
import numpy as np

logger = logging.getLogger(__name__)


class VideoStreamingService:
    """Service for streaming processed video frames with detections"""

    def __init__(self):
        self.current_frame = None
        self.quality = 80

    def write_frame(self, frame: np.ndarray):
        """Write a processed frame to be streamed (called by RGB detection service)"""
        self.current_frame = frame.copy()

    def set_quality(self, quality: int):
        """Set JPEG quality for streaming"""
        self.quality = max(1, min(100, quality))

    async def generate_frames(self) -> AsyncGenerator[bytes, None]:
        """Generate MJPEG frames from processed frames"""
        try:
            while True:
                if self.current_frame is not None:
                    # Encode frame as JPEG
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
                    success, buffer = cv2.imencode(
                        ".jpg", self.current_frame, encode_param
                    )

                    if success:
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n"
                            + buffer.tobytes()
                            + b"\r\n"
                        )
                        self.current_frame = None

                await asyncio.sleep(0.033)  # ~30 FPS

        except Exception as e:
            logger.error(f"Error in video streaming: {e}")
