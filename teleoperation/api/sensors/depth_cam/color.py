from .router import router
from fastapi import WebSocket, Response, WebSocketDisconnect
import asyncio
from fastapi.responses import HTMLResponse
import base64
from drivers.driver_dispatch import DriverDispatch

html = """
<!DOCTYPE html>
<html>
<head>
    <title>Top Camera Stream</title>
</head>
<body>
    <h1>Top Camera Stream</h1>
    <img id="video" src="">
    <script>
        var img = document.getElementById('video');
        var ws = new WebSocket("ws://er:8000/sensors/depth_camera/color_ws");
        ws.onmessage = function(event) {
            img.src = "data:image/jpeg;base64," + event.data;
        };
    </script>
</body>
</html>
"""

@router.get("/top_camera_video")
def get_top_camera_video():
    return HTMLResponse(html)

@router.get("/color.jpg")
async def get_color():
    with DriverDispatch.borrow("depth_camera") as depth_camera:
        img_bytes = depth_camera.get_jpg()
    return Response(content=img_bytes, media_type="image/jpeg")


@router.websocket("/color_ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            with DriverDispatch.borrow("depth_camera") as depth_camera:
                img_bytes = depth_camera.get_jpg()
            img_txt = base64.b64encode(img_bytes).decode("utf-8")
            await websocket.send_text(img_txt)
            await asyncio.sleep(0.05)
    except WebSocketDisconnect:
        print("client disconnected")
    except Exception as e:
        print(e)
    finally:
        await websocket.close()
