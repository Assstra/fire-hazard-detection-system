import argparse
from contextlib import asynccontextmanager
import time
from typing import Optional
from fastapi import FastAPI, HTTPException
from fastapi.responses import StreamingResponse
from sse_starlette.sse import EventSourceResponse
import logging

from lib.event_streaming import EventStreamer
from lib.video_writer import VideoWriterService
from lib.rgb_detection import RgbDetectionService
from lib.video_streaming import VideoStreamingService

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

rgb_video_writer: Optional[VideoWriterService] = None
rgb_detect_service: Optional[RgbDetectionService] = None
event_streamer: Optional[EventStreamer] = None
video_rgb_stream_service: Optional[VideoStreamingService] = None


def create_app(args: argparse.Namespace) -> FastAPI:
    """Create FastAPI application with detection service"""
    global rgb_video_writer, rgb_detect_service, event_streamer, video_rgb_stream_service

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        yield
        # Cleanup on shutdown
        if event_streamer:
            event_streamer.active_streams.clear()
        if rgb_video_writer:
            rgb_video_writer.release()
        logger.info("Server shutdown complete")

    if args.video_output:
        rgb_video_writer = VideoWriterService(
            output_path=f"{args.video_output}/{time.time_ns()}_rgb.mp4",
            frame_width=640,
            frame_height=480,
            fps=1,
        )

    video_rgb_stream_service = VideoStreamingService()

    # Create RGB detection service with video streaming service
    rgb_detect_service = RgbDetectionService(
        args.model, args.confidence, rgb_video_writer, video_rgb_stream_service
    )
    event_streamer = EventStreamer(rgb_detect_service)

    app = FastAPI(
        title="Fire/Smoke/Hot object Detection Server",
        version="1.0.0",
        lifespan=lifespan,
    )

    @app.get("/")
    async def root():
        return {
            "model_path": args.model,
            "endpoints": {
                "health": "/health",
                "events": "/events",
                "model_info": "/model/info",
                "video_stream": "/video/rgb",
            },
            "args": args.__dict__,
        }

    @app.get("/health")
    async def health_check():
        return {
            "status": "healthy",
            "model_loaded": rgb_detect_service.model is not None,
            "active_streams": len(event_streamer.active_streams),
        }

    @app.get("/model/info")
    async def model_info():
        if rgb_detect_service.model is None:
            raise HTTPException(status_code=500, detail="Model not loaded")

        print(f"Model path: {rgb_detect_service}")

        return {
            "model_path": rgb_detect_service.model_path,
            "class_names": rgb_detect_service.model.names,
            "input_size": getattr(rgb_detect_service.model, "imgsz", None),
        }

    @app.get("/events")
    async def stream_detections():
        """Stream detection events via SSE"""
        if rgb_detect_service is None:
            raise HTTPException(
                status_code=500, detail="Detection service not initialized"
            )

        return EventSourceResponse(
            event_streamer.stream_detections(args.video_input),
            media_type="text/event-stream",
        )

    @app.get("/video/rgb")
    async def video_stream(quality: int = 80):
        """Stream processed video with detections (MJPEG format)"""
        if video_rgb_stream_service is None:
            raise HTTPException(
                status_code=500, detail="Video streaming service not initialized"
            )

        # Set quality for streaming
        video_rgb_stream_service.set_quality(quality)

        return StreamingResponse(
            video_rgb_stream_service.generate_frames(),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    return app


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description="Fire/Smoke Detection Server")
    parser.add_argument(
        "--model",
        "-m",
        required=True,
        help="Path to YOLOv11 model file (.pt, .onnx, etc.)",
    )
    parser.add_argument(
        "--host", default="0.0.0.0", help="Host to bind to (default: 0.0.0.0)"
    )
    parser.add_argument(
        "--port", type=int, default=8000, help="Port to bind to (default: 8000)"
    )
    parser.add_argument(
        "--confidence",
        "-c",
        type=float,
        default=0.25,
        help="Sets the minimum confidence threshold for detections (default: 0.25)",
    )
    parser.add_argument(
        "--video-input",
        "-vi",
        default=0,
        help="Video input source (default: 0 for webcam, can be a file path or camera index)",
    )
    parser.add_argument(
        "--video-output",
        "-vo",
        type=str,
        default=None,
        help="Path to save the output video with detections (default: None, no video output)",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    app = create_app(args)

    import uvicorn

    logger.info(f"Starting server with model: {args.model}")
    logger.info(f"Server will be available at http://{args.host}:{args.port}")

    # Run server
    uvicorn.run(app, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
