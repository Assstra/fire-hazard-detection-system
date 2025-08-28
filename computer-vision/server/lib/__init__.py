import cv2
import numpy as np
from strenum import StrEnum


class DetectionKind(StrEnum):
    RGB = "rgb_detection"
    IR = "ir_detection"


class Position(StrEnum):
    """Enum to represent the position (left or right) from the center of the screen."""

    NONE = "none"
    LEFT = "left"
    RIGHT = "right"
    CENTER = "center"


def utils_frame_text(
    frame: np.ndarray,
    text: str,
    position: tuple[int, int],
    color: tuple[int, int, int] = (255, 255, 255),
    thickness: int = 2,
) -> np.ndarray:
    """Utility function to put text on the frame"""
    cv2.putText(
        frame,
        text,
        position,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        color,
        thickness,
    )
    return frame
