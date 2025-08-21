import requests
import json
import time
import argparse
from typing import Dict, Any


def parse_sse_event(line: str) -> Dict[str, Any]:
    """Parse SSE event line"""
    if line.startswith("data: "):
        try:
            return json.loads(line[6:])  # Remove 'data: ' prefix
        except json.JSONDecodeError:
            print(f"Failed to parse JSON for line: {line}")
            return {}
    return {}


def print_event(event_data: Dict[str, Any]):
    """Print event data in a formatted way"""
    event_type = event_data.get("type", "unknown")
    timestamp = time.strftime("%H:%M:%S")

    if event_type == "connected":
        stream_id = event_data.get("stream_id", "unknown")
        print(f"[{timestamp}] Connected to stream {stream_id}")

    elif event_type == "rgb_detection":
        fire_position = event_data.get("position", "unknown")
        timestamp_start = event_data.get("timestamp_start", "")
        frame_info = event_data.get("frame_info", {})

        print(f"[{timestamp}] Detection event:")
        print(f"  Frame: {frame_info.get('width', 0)}x{frame_info.get('height', 0)}")
        print(f"  Fire position: {fire_position}")
        print(f"  Elapsed time: {time.time() - float(timestamp_start):.4f} seconds")

    elif event_type == "ir_detection":
        pass
        # fire_position = event_data.get("position", "unknown")
        # timestamp_start = event_data.get("timestamp_start", "")
        # frame_info = event_data.get("frame_info", {})

        # print(f"[{timestamp}] Detection event:")
        # print(f"  Frame: {frame_info.get('width', 0)}x{frame_info.get('height', 0)}")
        # print(f"  Fire position: {fire_position}")
        # print(f"  Elapsed time: {time.time() - float(timestamp_start):.4f} seconds")

    elif event_type == "error":
        message = event_data.get("message", "Unknown error")
        stream_id = event_data.get("stream_id", "unknown")
        print(f"[{timestamp}] Error in stream {stream_id}: {message}")

    else:
        print(f"[{timestamp}] Unknown event type: {event_type}")


def test_server_endpoints(base_url: str):
    """Test basic server endpoints"""
    print("Testing server endpoints...")

    # Test root endpoint
    try:
        response = requests.get(f"{base_url}/")
        if response.status_code == 200:
            print("✓ Root endpoint working")
        else:
            print(f"✗ Root endpoint failed: {response.status_code}")
    except Exception as e:
        print(f"✗ Root endpoint error: {e}")

    # Test health endpoint
    try:
        response = requests.get(f"{base_url}/health")
        if response.status_code == 200:
            health_data = response.json()
            print(
                f"✓ Health endpoint working - Status: {health_data.get('status', 'unknown')}"
            )
            print(f"  Model loaded: {health_data.get('model_loaded', False)}")
            print(f"  Active streams: {health_data.get('active_streams', 0)}")
        else:
            print(f"✗ Health endpoint failed: {response.status_code}")
    except Exception as e:
        print(f"✗ Health endpoint error: {e}")

    # Test model info endpoint
    try:
        response = requests.get(f"{base_url}/model/info")
        if response.status_code == 200:
            model_info = response.json()
            print("✓ Model info endpoint working")
            print(f"  Model path: {model_info.get('model_path', 'unknown')}")
            print(f"  Classes: {list(model_info.get('class_names', {}).values())}")
        else:
            print(f"✗ Model info endpoint failed: {response.status_code}")
    except Exception as e:
        print(f"✗ Model info endpoint error: {e}")

    print()


def stream_detections(base_url: str, video_source: int = 0):
    """Connect to SSE stream and display detection events"""
    stream_url = f"{base_url}/events"

    if video_source != 0:
        stream_url += f"?video_source={video_source}"

    print(f"Connecting to detection stream: {stream_url}")
    print("Press Ctrl+C to stop\n")

    try:
        # Connect to SSE stream
        response = requests.get(stream_url, stream=True, timeout=30)
        response.raise_for_status()

        # Process SSE events
        for line in response.iter_lines(decode_unicode=True):
            if line:
                event_data = parse_sse_event(line)
                if event_data:
                    print_event(event_data)

    except KeyboardInterrupt:
        print("\nStopping client...")
    except requests.exceptions.RequestException as e:
        print(f"Connection error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description="Test client for Fire/Smoke/Hot objects Detection Server"
    )
    parser.add_argument(
        "--host", default="localhost", help="Server host (default: localhost)"
    )
    parser.add_argument(
        "--port", type=int, default=8000, help="Server port (default: 8000)"
    )
    parser.add_argument(
        "--video-source",
        type=int,
        default=0,
        help="Video source index (default: 0 for webcam)",
    )
    parser.add_argument(
        "--no-streaming", action="store_true", help="Only test endpoints, don't stream"
    )

    args = parser.parse_args()

    base_url = f"http://{args.host}:{args.port}"

    print("Fire/Smoke/Hot objects Detection Server Test Client")
    print(f"Server: {base_url}")
    print("=" * 50)

    # Test basic endpoints
    test_server_endpoints(base_url)

    if not args.no_streaming:
        # Start streaming
        stream_detections(base_url, args.video_source)


if __name__ == "__main__":
    main()
