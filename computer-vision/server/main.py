import argparse
from contextlib import asynccontextmanager
from typing import Optional
from fastapi import FastAPI, HTTPException
from sse_starlette.sse import EventSourceResponse
import logging

from lib.event_streaming import EventStreamer
from lib.yolo_detection import YOLODetectionService

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

yolo_detect_service: Optional[YOLODetectionService] = None
event_streamer: Optional[EventStreamer] = None


def create_app(model_path: str) -> FastAPI:
    """Create FastAPI application with detection service"""
    global yolo_detect_service, event_streamer

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        yield
        # Cleanup on shutdown
        if event_streamer:
            event_streamer.active_streams.clear()
        logger.info("Server shutdown complete")

    yolo_detect_service = YOLODetectionService(model_path)
    event_streamer = EventStreamer(yolo_detect_service)

    app = FastAPI(
        title="Fire/Smoke/Hot object Detection Server",
        version="1.0.0",
        lifespan=lifespan,
    )

    @app.get("/")
    async def root():
        return {
            "message": app.title,
            "model_path": model_path,
            "endpoints": {
                "health": "/health",
                "events": "/events",
                "model_info": "/model/info",
            },
        }

    @app.get("/health")
    async def health_check():
        return {
            "status": "healthy",
            "model_loaded": yolo_detect_service.model is not None,
            "active_streams": len(event_streamer.active_streams),
        }

    @app.get("/model/info")
    async def model_info():
        if yolo_detect_service.model is None:
            raise HTTPException(status_code=500, detail="Model not loaded")

        print(f"Model path: {yolo_detect_service}")

        return {
            "model_path": yolo_detect_service.model_path,
            "class_names": yolo_detect_service.model.names,
            "input_size": getattr(yolo_detect_service.model, "imgsz", None),
        }

    @app.get("/events")
    async def stream_detections(video_source: int = 0):
        """Stream detection events via SSE"""
        if yolo_detect_service is None:
            raise HTTPException(
                status_code=500, detail="Detection service not initialized"
            )

        return EventSourceResponse(
            event_streamer.stream_detections(video_source),
            media_type="text/event-stream",
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
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Log level (default: INFO)",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    logging.getLogger().setLevel(getattr(logging, args.log_level))

    app = create_app(args.model)

    import uvicorn

    logger.info(f"Starting server with model: {args.model}")
    logger.info(f"Server will be available at http://{args.host}:{args.port}")
    logger.info("Detection event stream endpoint: /events")

    # Run server
    uvicorn.run(app, host=args.host, port=args.port, log_level=args.log_level.lower())


if __name__ == "__main__":
    main()
