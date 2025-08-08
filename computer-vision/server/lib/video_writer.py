import logging
import cv2
import numpy as np


logger = logging.getLogger(__name__)


class VideoWriterService:
    """Service for writing video frames to a file"""

    def __init__(self, output_path: str, frame_width: int, frame_height: int, fps: int):
        self.video_width = frame_width
        self.video_height = frame_height
        self.video_writer = cv2.VideoWriter(
            output_path,
            cv2.VideoWriter_fourcc(*"mp4v"),
            fps,
            (frame_width, frame_height),
        )
        logger.info(
            f"Video writer with resolution {frame_width}x{frame_height} initialized for {output_path} with FPS: {fps}"
        )

    def write_frame(self, frame: np.ndarray):
        """Write a single frame to the video file"""
        self.video_writer.write(frame)

    def release(self):
        """Release the video writer resources"""
        self.video_writer.release()
        logger.info("Video writer released")

    def get_width(self) -> int:
        """Get the width of the video frames"""
        return self.video_width

    def get_height(self) -> int:
        """Get the height of the video frames"""
        return self.video_height
