from contextlib import asynccontextmanager
import time
from typing import Optional
import cv2
from fastapi import FastAPI, HTTPException
from fastapi.responses import StreamingResponse
from sse_starlette.sse import EventSourceResponse
import logging

from lib.config import Config
from lib.event_streaming import EventStreamer
from lib.video_writer import VideoWriterService
from lib.rgb_detection import RgbDetectionService
from lib.ir_detection import InfraredDetectionService
from lib.video_streaming import VideoStreamingService

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

raw_rgb_video_writer: Optional[VideoWriterService] = None
rgb_video_writer: Optional[VideoWriterService] = None
rgb_video_stream: Optional[VideoStreamingService] = None
rgb_detect_service: Optional[RgbDetectionService] = None
raw_ir_video_writer: Optional[VideoWriterService] = None
ir_video_writer: Optional[VideoWriterService] = None
ir_video_stream: Optional[VideoStreamingService] = None
ir_detect_service: Optional[InfraredDetectionService] = None
event_streamer: Optional[EventStreamer] = None


def create_app(config: Config) -> FastAPI:
    global \
        raw_rgb_video_writer, \
        rgb_video_writer, \
        rgb_video_stream, \
        rgb_detect_service, \
        raw_ir_video_writer, \
        ir_video_writer, \
        ir_video_stream, \
        ir_detect_service, \
        event_streamer

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        yield
        # Cleanup on shutdown
        if event_streamer:
            event_streamer.active_streams.clear()
        if raw_rgb_video_writer:
            raw_rgb_video_writer.release()
        if rgb_video_writer:
            rgb_video_writer.release()
        if raw_ir_video_writer:
            raw_ir_video_writer.release()
        if ir_video_writer:
            ir_video_writer.release()
        if ir_detect_service:
            ir_detect_service.close()
        logger.info("Server shutdown complete")

    if config.video_output:
        video_name = time.time_ns()
        raw_rgb_video_writer = VideoWriterService(
            output_path=f"{config.video_output}/{video_name}_rgb_raw.mp4",
            frame_width=640,
            frame_height=480,
            fps=10,
        )
        rgb_video_writer = VideoWriterService(
            output_path=f"{config.video_output}/{video_name}_rgb.mp4",
            frame_width=640,
            frame_height=480,
            fps=10,
        )
        if not config.disable_ir:
            raw_ir_video_writer = VideoWriterService(
                output_path=f"{config.video_output}/{video_name}_ir_raw.mkv",
                frame_width=160,
                frame_height=120,
                fps=10,
                codec=cv2.VideoWriter_fourcc(*"xvid"),
            )
            ir_video_writer = VideoWriterService(
                output_path=f"{config.video_output}/{video_name}_ir.mp4",
                frame_width=160,
                frame_height=120,
                fps=10,
            )

    rgb_video_stream = VideoStreamingService()
    rgb_detect_service = RgbDetectionService(
        config.model,
        config.confidence,
        raw_rgb_video_writer,
        rgb_video_writer,
        rgb_video_stream,
    )
    if not config.disable_ir:
        ir_video_stream = VideoStreamingService()
        ir_detect_service = InfraredDetectionService(
            raw_ir_video_writer, ir_video_writer, ir_video_stream
        )

    event_streamer = EventStreamer(rgb_detect_service, ir_detect_service)

    app = FastAPI(
        title="Fire/Smoke/Hot object Detection Server",
        version="1.0.0",
        lifespan=lifespan,
    )

    @app.get("/")
    async def root():
        return {
            "model_path": config.model,
            "args": config.__dict__,
            "class_names": rgb_detect_service.model.names,
            "input_size": getattr(rgb_detect_service.model, "imgsz", None),
        }

    @app.get("/health")
    async def health_check():
        return {
            "status": "healthy",
            "active_streams": len(event_streamer.active_streams),
        }

    @app.get("/events")
    async def stream_detections():
        """Stream detection events via SSE"""
        if rgb_detect_service is None:
            raise HTTPException(
                status_code=500, detail="Detection service not initialized"
            )

        return EventSourceResponse(
            event_streamer.stream_detections(config.video_input),
            media_type="text/event-stream",
        )

    @app.get("/video/rgb")
    async def video_stream_for_rgb(quality: int = 60):
        """Stream processed video with RGB detections"""
        if rgb_video_stream is None:
            raise HTTPException(
                status_code=500, detail="Video streaming service not initialized"
            )
        rgb_video_stream.set_quality(quality)

        return StreamingResponse(
            rgb_video_stream.generate_frames(),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    @app.get("/video/ir")
    async def video_stream_for_ir(quality: int = 60):
        """Stream processed video with IR detections"""
        if config.disable_ir:
            raise HTTPException(
                status_code=403, detail="IR streaming is disabled by configuration"
            )

        if ir_video_stream is None:
            raise HTTPException(
                status_code=500, detail="Video streaming service not initialized"
            )
        ir_video_stream.set_quality(quality)

        return StreamingResponse(
            ir_video_stream.generate_frames(),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    return app


def main():
    config = Config.from_env()
    logger.info(f"Configuration loaded: {config.__dict__}")

    app = create_app(config)

    import uvicorn

    logger.info(f"Starting server with model: {config.model}")
    logger.info(f"Server will be available at http://{config.host}:{config.port}")

    # Run server
    uvicorn.run(app, host=config.host, port=config.port)


if __name__ == "__main__":
    main()
