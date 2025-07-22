from enum import StrEnum


class DetectionKind(StrEnum):
    RGB = "rgb_detection"


class Position(StrEnum):
    """Enum to represent the position (left or right) from the center of the screen."""

    NONE = "none"
    LEFT = "left"
    RIGHT = "right"
    CENTER = "center"
