import os
from typing import Optional


class Config:
    def __init__(
        self,
        host: str,
        port: int,
        model: str,
        video_input: int | str,
        video_output: Optional[str] = None,
        confidence: float = 0.25,
        center_threshold: int = 75,
        disable_ir: bool = False,
    ):
        self.host = host
        self.port = port
        self.model = model
        self.video_input = video_input
        self.video_output = video_output
        self.confidence = confidence
        self.center_threshold = center_threshold
        self.disable_ir = disable_ir

    @classmethod
    def from_env(cls):
        video_input = os.getenv("VIDEO_INPUT", 0)
        if isinstance(video_input, str) and video_input.isdigit():
            video_input = int(video_input)

        return cls(
            host=os.getenv("HOST", "0.0.0.0"),
            port=int(os.getenv("PORT", 8000)),
            model=os.getenv("MODEL_PATH", "/opt/models/model.pt"),
            video_input=video_input,
            video_output=os.getenv("VIDEO_OUTPUT", None),
            confidence=float(os.getenv("CONFIDENCE", 0.25)),
            center_threshold=int(os.getenv("CENTER_THRESHOLD", 75)),
            disable_ir=isinstance(os.getenv("DISABLE_IR"), str),
        )
