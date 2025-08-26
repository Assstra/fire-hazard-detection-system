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
MODEL_PATH="path/to/your/model.pt" uv run python main.py
```

You can use the model trained in the `../training` directory.

### Configuration

Via environment variables:

- `HOST`: Host to bind to (default: `0.0.0.0`)
- `PORT`: Port to bind to (default: `8000`)
- `MODEL_PATH`: (default: `/opt/models/model.pt`)  
  Path to YOLOv11 model file, can be in various formats:
  - `.pt` (PyTorch)
  - `.onnx` (ONNX)
  - ~~`.engine` (TensorRT)~~ todo: currently not supported... issues with tensorRT versions
- `VIDEO_INPUT`: `device_id` (like `0` or `1`), or a filename to a video (default: `0`)
- `VIDEO_OUTPUT`: Path to a folder to save video output (default: `None`, no video output)
- `CONFIDENCE`: Sets the minimum confidence threshold for detections (default: `0.25`)
- `CENTER_THRESHOLD`: Sets the center threshold for position detection (default: `75`)
- `DISABLE_IR`: If you put anything in this variable, it will disable the IR camera

### Docker Usage

> [!NOTE]
> This program is designed to run on NVIDIA Jetson devices (based on the [`nvcr.io/nvidia/l4t-jetpack`](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-jetpack) image).
> 
> For now, there is no support for other platforms.

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
Returns server information.

#### Health Check
```
GET /health
```
Returns server health status and active stream count.

#### Detection Stream (SSE)
```
GET /events
```
Streams detection events in real-time via Server-Sent Events.

#### Live real-time Video Stream

For RGB camera:

```
GET /video/rgb
```

For IR camera:

```
GET /video/ir
```

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
uv run python test_client.py
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
  // or `ir_detection`
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
