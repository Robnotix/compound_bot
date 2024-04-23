from .router import router
from fastapi import WebSocket, Response, WebSocketDisconnect
from fastapi.responses import HTMLResponse
import asyncio
from threading import Lock
import base64
import atexit

from sensing.cameras.front_facing import FrontFacingCamera

front_camera = FrontFacingCamera()
front_camera.start()
device_lock = Lock()

atexit.register(front_camera.stop)

html = """
<!DOCTYPE html>
<html>
<head>
    <title>Front Camera Stream</title>
</head>
<body>
    <h1>Front Camera Stream</h1>
    <img id="video" src="">
    <script>
        var img = document.getElementById('video');
        var ws = new WebSocket("ws://er:8000/sensors/front_camera_ws");
        ws.onmessage = function(event) {
            img.src = "data:image/jpeg;base64," + event.data;
        };
    </script>
</body>
</html>
"""

@router.get("/front_camera_video")
def get_front_camera_video():
    return HTMLResponse(html)


# make a websocket that serves the latest image
@router.websocket("/front_camera_ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            with device_lock:
                img_bytes = front_camera.get_jpg()
            img_txt = base64.b64encode(img_bytes).decode("utf-8")
            await websocket.send_text(img_txt)
            await asyncio.sleep(0.05)
    except WebSocketDisconnect:
        print("client disconnected")
    except Exception as e:
        print(e)
    finally:
        await websocket.close()

@router.get("/front_camera.jpg")
def get_front_camera():
    """
    returns the most current image from the front camera
    """
    with device_lock:
        img_bytes = front_camera.get_jpg()
    return Response(content=img_bytes, media_type="image/jpeg")
