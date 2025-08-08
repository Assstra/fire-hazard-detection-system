import asyncio
import logging
from typing import AsyncGenerator
from sse_starlette import JSONServerSentEvent
from lib.rgb_detection import RgbDetectionService

logger = logging.getLogger(__name__)


class EventStreamer:
    """Handles SSE event streaming for detection results"""

    def __init__(self, detection_service: RgbDetectionService):
        self.detection_service = detection_service
        self.active_streams = set()

    async def stream_detections(
        self, video_source: int = 0
    ) -> AsyncGenerator[JSONServerSentEvent, None]:
        """Stream detection events as SSE"""
        stream_id = id(asyncio.current_task())
        self.active_streams.add(stream_id)

        try:
            logger.info(f"Starting detection stream {stream_id}")

            # Send initial connection event
            yield JSONServerSentEvent(
                data={"type": "connected", "stream_id": stream_id},
            )

            # Start detection generator
            detection_gen = self.detection_service.detect_from_video_stream(
                video_source
            )

            async for detection_event in detection_gen:
                yield JSONServerSentEvent(data=detection_event)

        except Exception as e:
            logger.error(f"Error in detection stream {stream_id}: {e}")
            yield JSONServerSentEvent(
                data={"type": "error", "message": f"{e}", "stream_id": stream_id}
            )

        finally:
            self.active_streams.discard(stream_id)
            logger.info(f"Closed detection stream {stream_id}")
