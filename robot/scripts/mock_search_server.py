import json
import time
import asyncio
from fastapi import FastAPI, Request, Form
from fastapi.responses import StreamingResponse, HTMLResponse, RedirectResponse

app = FastAPI()

clients = []  # List of asyncio queues for each client

# Helper to format SSE
def sse_format(data):
    return f"data: {json.dumps(data)}\n\n"

@app.get("/", response_class=HTMLResponse)
async def root():
    return '''
        <h2>Send SSE Event</h2>
        <form method="post" action="/send">
            <label>Type:</label>
            <select name="type">
                <option value="rgb_detection">rgb_detection</option>
                <option value="connected">connected</option>
                <option value="error">error</option>
            </select><br>
            <label>Stream ID:</label><input name="stream_id" value="mock_stream_1"><br>
            <label>Message (for error):</label><input name="message"><br>
            <label>Position (for detection):</label>
            <select name="position">
                <option value="left">left</option>
                <option value="right">right</option>
                <option value="center">center</option>
                <option value="none">none</option>
            </select><br>
            <label>Timestamp Start (for detection):</label><input name="timestamp_start" type="number" step="any" value="0"><br>
            <label>Frame Width:</label><input name="frame_width" type="number" value="640"><br>
            <label>Frame Height:</label><input name="frame_height" type="number" value="480"><br>
            <label>Frame Channels:</label><input name="frame_channels" type="number" value="3"><br>
            <button type="submit">Send</button>
        </form>
        <p>Connect your SSE client to <code>/events</code></p>
    '''

@app.post("/send")
async def send_event(
    type: str = Form(...),
    stream_id: str = Form("mock_stream_1"),
    message: str = Form(""),
    position: str = Form("none"),
    timestamp_start: float = Form(0),
    frame_width: int = Form(640),
    frame_height: int = Form(480),
    frame_channels: int = Form(3)
):
    if type == "connected":
        data = {"type": "connected", "stream_id": stream_id}
    elif type == "error":
        data = {"type": "error", "message": message, "stream_id": stream_id}
    elif type == "rgb_detection":
        data = {
            "type": "rgb_detection",
            "position": position,
            "timestamp_start": float(timestamp_start) if timestamp_start else time.time(),
            "frame_info": {
                "width": frame_width,
                "height": frame_height,
                "channels": frame_channels
            }
        }
    else:
        data = {"type": type, "message": message}
    # Broadcast to all clients
    for q in clients:
        await q.put(data)
    return RedirectResponse(url="/", status_code=303)

@app.get("/events")
async def events(request: Request):
    q = asyncio.Queue()
    clients.append(q)
    async def event_stream():
        # Initial event
        yield sse_format({"type": "connected", "stream_id": "mock_stream_1"})
        while True:
            if await request.is_disconnected():
                break
            data = await q.get()
            yield sse_format(data)
    return StreamingResponse(event_stream(), media_type="text/event-stream")

# To run: uvicorn mock_search_server:app --reload --port 5000
