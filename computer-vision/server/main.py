import argparse
import asyncio
from contextlib import asynccontextmanager
import time
from typing import AsyncGenerator, Optional, Dict, Any, List
import cv2
import numpy as np
from fastapi import FastAPI, HTTPException
from sse_starlette import JSONServerSentEvent
from ultralytics import YOLO
from sse_starlette.sse import EventSourceResponse
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class YOLODetectionService:
    """Service for fire/smoke detection using YOLOv11 model"""

    def __init__(self, model_path: str):
        self.model_path = model_path
        self.model = None
        self.load_model()

    def load_model(self):
        """Load YOLOv11 model"""
        try:
            self.model = YOLO(self.model_path, task="detect")
            logger.info(f"Successfully loaded model from {self.model_path}")
        except Exception as e:
            logger.error(f"Failed to load model: {e}")
            raise HTTPException(status_code=500, detail=f"Failed to load model: {e}")

    def detect_from_image(self, image: np.ndarray) -> List[Dict[str, Any]]:
        """Detect in image"""
        if self.model is None:
            raise HTTPException(status_code=500, detail="Model not loaded")

        try:
            results = self.model.predict(image, stream=True, conf=0.5)
            detections = []

            for result in results:
                boxes = result.boxes
                if boxes is None:
                    continue
                for box in boxes:
                    # Extract detection info
                    conf = float(box.conf.cpu().numpy()[0])
                    cls = int(box.cls.cpu().numpy()[0])
                    xyxy = box.xyxy.cpu().numpy()[0]

                    # Get class name
                    class_name = (
                        self.model.names[cls]
                        if cls < len(self.model.names)
                        else "unknown"
                    )

                    detection = {
                        "class": class_name,
                        "confidence": conf,
                        "bbox": {
                            "x1": float(xyxy[0]),
                            "y1": float(xyxy[1]),
                            "x2": float(xyxy[2]),
                            "y2": float(xyxy[3]),
                        },
                        "timestamp": time.time(),
                    }
                    detections.append(detection)

            return detections

        except Exception as e:
            logger.error(f"Detection error: {e}")
            return []

    def detect_from_video_stream(
        self, video_source: int = 0
    ) -> AsyncGenerator[Dict[str, Any], None]:
        """Generator for continuous video detection"""

        async def detection_generator():
            cap = cv2.VideoCapture(video_source)

            if not cap.isOpened():
                logger.error(f"Cannot open video source: {video_source}")
                return

            try:
                while True:
                    ret, frame = cap.read()
                    if not ret:
                        logger.warning("Failed to read frame")
                        break

                    # Detect objects in frame
                    detections = self.detect_from_image(frame)

                    # Yield detection event if any detections found
                    if detections:
                        event_data = {
                            "type": "yolo_detection",
                            "detections": detections,
                            "frame_info": {
                                "width": frame.shape[1],
                                "height": frame.shape[0],
                                "channels": frame.shape[2],
                            },
                        }
                        yield event_data

                    # Small delay to prevent overwhelming the client
                    await asyncio.sleep(1)

            finally:
                cap.release()

        return detection_generator()


class EventStreamer:
    """Handles SSE event streaming for detection results"""

    def __init__(self, detection_service: YOLODetectionService):
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
                if stream_id not in self.active_streams:
                    break

                yield JSONServerSentEvent(data=detection_event)

        except Exception as e:
            logger.error(f"Error in detection stream {stream_id}: {e}")
            yield JSONServerSentEvent(
                data={"type": "error", "message": f"{e}", "stream_id": stream_id}
            )

        finally:
            self.active_streams.discard(stream_id)
            logger.info(f"Closed detection stream {stream_id}")


# Global detection service instance
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
