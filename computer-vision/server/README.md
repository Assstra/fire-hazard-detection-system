# Fire/Smoke Detection Server

This server provides real-time fire, smoke and hot object detection using a YOLOv11 models and an IR camera, with Server-Sent Events (SSE) streaming.

## Installation

Install the dependencies:

```bash
uv sync
```

## Usage

### Running the Server

The server requires a trained YOLOv11 model file.

```bash
# Basic usage with model file
uv run python main.py --model path/to/your/model.pt
```

You can use the model trained in the `../training` directory.

### Command Line Arguments

- `--model, -m`: **Required** - Path to YOLOv11 model file, can be in various formats:
  - `.pt` (PyTorch)
  - `.onnx` (ONNX)
  - `.engine` (TensorRT)
- `--host`: Host to bind to (default: `0.0.0.0`)
- `--port`: Port to bind to (default: `8000`)
- `--confidence`: Sets the minimum confidence threshold for detections (default: 0.25)

### Docker Usage

You can run the server using Docker:

```bash
docker compose up
```

If you want GPU support, uncomment the `runtime`, `ipc`, and `ulimits` lines in the `compose.yaml` file.

See [Enable GPU support in Docker](https://docs.docker.com/compose/how-tos/gpu-support/) for more information.

### API Endpoints

#### Root Endpoint
```
GET /
```
Returns server information and available endpoints.

#### Health Check
```
GET /health
```
Returns server health status and active stream count.

#### Model Information
```
GET /model/info
```
Returns loaded model details including class names and configuration.

#### Detection Stream (SSE)
```
GET /events?video_source=0
```
Streams detection events in real-time via Server-Sent Events.

Query parameters:
- `video_source`: Video source index (default: 0 for webcam)

## Testing

Use the provided test client to verify the server:

```bash
# Test all endpoints
uv run python test_client.py

# Test with custom server
uv run python test_client.py --host localhost --port 8000

# Only test endpoints (no streaming)
uv run python test_client.py --no-streaming

# Stream from different video source
uv run python test_client.py --video-source 1
```

## Event Data Types

The SSE stream produces the following event data types:

### Connection Event

```json
{
  "type": "connected",
  "stream_id": "unique_stream_id"
}
```

### Detection Event

```jsonc
{
  "type": "rgb_detection",
  // Position of the boxes, from the center: "left", "right", "center" or "none"
  "position": "left",
  // If you want to measure the time spent in detection
  "timestamp_start": 1753158175.0682974,
  "frame_info": {
    "width": 640,
    "height": 480,
    "channels": 3
  }
}
```

### Error Event

```json
{
  "type": "error",
  "message": "Error description",
  "stream_id": "unique_stream_id"
}
```
