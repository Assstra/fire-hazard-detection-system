from multiprocessing.connection import Connection
import requests
import json
import time
from typing import Dict, Any

import rospy


def parse_sse_event(line: str) -> Dict[str, Any]:
    """Parse SSE event line"""
    if line.startswith("data: "):
        try:
            return json.loads(line[6:])  # Remove 'data: ' prefix
        except json.JSONDecodeError:
            rospy.logerr(f"Failed to parse JSON for line: {line}")
            return {}
    return {}


def print_event(event_data: Dict[str, Any]):
    """Print event data in a formatted way, or send via pipe if provided"""
    event_type = event_data.get("type", "unknown")
    timestamp = time.strftime("%H:%M:%S")

    if event_type == "connected":
        stream_id = event_data.get("stream_id", "unknown")

    elif event_type == "error":
        message = event_data.get("message", "Unknown error")
        stream_id = event_data.get("stream_id", "unknown")
        rospy.logwarn(f"[{timestamp}] Error in stream {stream_id}: {message}")

    else:
        rospy.loginfo(f"[{timestamp}] Unknown event type: {event_type}")


def test_server_endpoints(base_url: str):
    """Test basic server endpoints"""
    rospy.loginfo("Testing server endpoints...")

    # Test root endpoint
    try:
        response = requests.get(f"{base_url}/")
        if response.status_code == 200:
            rospy.loginfo("✓ Root endpoint working")
        else:
            rospy.logwarn(f"✗ Root endpoint failed: {response.status_code}")
    except Exception as e:
        rospy.logerr(f"✗ Root endpoint error: {e}")

    # Test health endpoint
    try:
        response = requests.get(f"{base_url}/health")
        if response.status_code == 200:
            health_data = response.json()
            rospy.loginfo(
                f"✓ Health endpoint working - Status: {health_data.get('status', 'unknown')}"
            )
            rospy.loginfo(f"  Model loaded: {health_data.get('model_loaded', False)}")
            rospy.loginfo(f"  Active streams: {health_data.get('active_streams', 0)}")
        else:
            rospy.logwarn(f"✗ Health endpoint failed: {response.status_code}")
    except Exception as e:
        rospy.logerr(f"✗ Health endpoint error: {e}")

    # Test model info endpoint
    try:
        response = requests.get(f"{base_url}/model/info")
        if response.status_code == 200:
            model_info = response.json()
            rospy.loginfo("✓ Model info endpoint working")
            rospy.loginfo(f"  Model path: {model_info.get('model_path', 'unknown')}")
            rospy.loginfo(f"  Classes: {list(model_info.get('class_names', {}).values())}")
        else:
            rospy.logwarn(f"✗ Model info endpoint failed: {response.status_code}")
    except Exception as e:
        rospy.logerr(f"✗ Model info endpoint error: {e}")

    print()


def stream_detections(base_url: str, pipe: Connection = None, video_source: int = 0):
    """Connect to SSE stream and display detection events, or send via pipe if provided"""
    stream_url = f"{base_url}/events"

    if video_source != 0:
        stream_url += f"?video_source={video_source}"

    if pipe is None:
        rospy.loginfo(f"Connecting to detection stream: {stream_url}")
        rospy.loginfo("Press Ctrl+C to stop\n")

    try:
        # Connect to SSE stream
        response = requests.get(stream_url, stream=True, timeout=30)
        response.raise_for_status()

        # Process SSE events
        for line in response.iter_lines(decode_unicode=True):
            if line:
                event_data = parse_sse_event(line)
                if event_data:
                    if pipe is not None:
                        pipe.send(event_data)
                    print_event(event_data)

    except KeyboardInterrupt:
        if pipe is None:
            rospy.loginfo("\nStopping client...")
    except requests.exceptions.RequestException as e:
        if pipe is not None:
            pipe.send({"type": "error", "message": str(e)})
        else:
            rospy.logerr(f"Connection error: {e}")
    except Exception as e:
        if pipe is not None:
            pipe.send({"type": "error", "message": str(e)})
        else:
            rospy.logerr(f"Unexpected error: {e}")


def search(host: str, port: int, pipe: Connection = None):
    base_url = f"http://{host}:{port}"

    if pipe is None:
        rospy.loginfo("Fire/Smoke/Hot objects Detection Server Test Client")
        rospy.loginfo(f"Server: {base_url}")
        rospy.loginfo("=" * 50)

    # Test basic endpoints
    test_server_endpoints(base_url)

    stream_detections(base_url, pipe)
