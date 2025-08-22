import asyncio
import logging
from typing import AsyncGenerator, Optional
from sse_starlette import JSONServerSentEvent
from lib import DetectionKind
from lib.rgb_detection import RgbDetectionService
from lib.ir_detection import InfraredDetectionService

logger = logging.getLogger(__name__)


class EventStreamer:
    """Handles SSE event streaming for detection results"""

    def __init__(
        self,
        rgb_detection_service: RgbDetectionService,
        ir_detection_service: Optional[InfraredDetectionService],
    ):
        self.rgb_detection_service = rgb_detection_service
        self.ir_detection_service = ir_detection_service
        self.active_streams = set()

    async def stream_detections(
        self, rgb_video_source: int = 0
    ) -> AsyncGenerator[JSONServerSentEvent, None]:
        """Stream detection events as SSE"""
        stream_id = id(asyncio.current_task())
        self.active_streams.add(stream_id)
        tasks = []

        try:
            logger.info(f"Starting detection stream {stream_id}")

            # Send initial connection event
            yield JSONServerSentEvent(
                data={"type": "connected", "stream_id": stream_id},
            )

            rgb_generator = self.rgb_detection_service.detect_from_video_stream(
                rgb_video_source
            )
            if self.ir_detection_service is not None:
                ir_generator = self.ir_detection_service.detect_from_video_stream()

            event_queue = asyncio.Queue()

            tasks = []
            rgb_task = asyncio.create_task(
                self.stream_consumer(rgb_generator, DetectionKind.RGB, event_queue)
            )
            tasks.append(rgb_task)

            if self.ir_detection_service is not None:
                ir_task = asyncio.create_task(
                    self.stream_consumer(ir_generator, DetectionKind.IR, event_queue)
                )
                tasks.append(ir_task)

            while True:
                try:
                    event_data = await asyncio.wait_for(event_queue.get(), timeout=1.0)
                    yield JSONServerSentEvent(data=event_data)
                except asyncio.TimeoutError:
                    continue

        except Exception as e:
            logger.error(f"Error in detection stream {stream_id}: {e}")
            yield JSONServerSentEvent(
                data={"type": "error", "message": f"{e}", "stream_id": stream_id}
            )

        finally:
            # Cancel all background tasks when stream ends
            for task in tasks:
                if not task.done():
                    task.cancel()
                    try:
                        await task
                    except asyncio.CancelledError:
                        logger.info(
                            f"Successfully cancelled task for stream {stream_id}"
                        )
                    except Exception as e:
                        logger.error(
                            f"Error cancelling task for stream {stream_id}: {e}"
                        )

            self.active_streams.discard(stream_id)
            logger.info(f"Closed detection stream {stream_id}")

    async def stream_consumer(self, generator, source_type, queue):
        """Consume events from a generator and put them in the queue"""
        try:
            async for event in generator:
                await queue.put(event)
        except asyncio.CancelledError:
            logger.info(f"{source_type} stream consumer cancelled")
            raise
        except Exception as e:
            logger.error(f"Error in {source_type} stream consumer: {e}")
            await queue.put({"type": "error", "message": str(e)})
