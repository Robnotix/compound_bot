from sensing.lidar import LidarScan, LidarX2
from .router import router
from threading import Lock
import atexit
from fastapi import WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
import asyncio

DEFAULT_PORT = "/dev/ttyAMA0"
lidar = LidarX2(DEFAULT_PORT)
device_lock = Lock()

if not lidar.open():
    raise Exception("Failed to open lidar")

atexit.register(lidar.close)

html = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Lidar Data Viewer</title>
    <style>
        canvas {
            border: 1px solid black;
        }
    </style>
</head>
<body>
    <h1>Lidar Data Streaming</h1>
    <canvas id="lidarCanvas" width="600" height="600"></canvas>
    <script>
        var canvas = document.getElementById('lidarCanvas');
        var ctx = canvas.getContext('2d');
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;

        var ws = new WebSocket("ws://er:8000/sensors/lidar_ws");
        ws.onmessage = function(event) {
            var data = JSON.parse(event.data);
            var measurements = data.measurements;
            drawLidarData(measurements);
        };

        function drawLidarData(measurements) {
            ctx.clearRect(0, 0, canvas.width, canvas.height);  // Clear the canvas for each new scan
            // get min and max values for x and y
            var maxDist = Number.MIN_VALUE;
            
            measurements.forEach((measurement) => {
                var distance = measurement[1];  // Distance
                maxDist = Math.max(maxDist, distance);
            });
            measurements.forEach((measurement) => {
                var angle = measurement[0];  // Angle in degrees
                var distance = canvas.width * measurement[1] / maxDist;  // Distance (scaled)
                var angleRad = angle * Math.PI / 180;  // Convert degrees to radians

                var x = centerX + distance * Math.cos(angleRad);
                var y = centerY + distance * Math.sin(angleRad);

                ctx.beginPath();
                ctx.arc(x, y, 2, 0, 2 * Math.PI, false);  // Draw a small circle for each measurement
                ctx.fill();
            });

        }

        ws.onclose = function() {
            console.log("WebSocket connection closed");
        };
    </script>
</body>
</html>
"""

@router.get("/lidar_view")
def view_lidar():
    return HTMLResponse(html)

@router.websocket("/lidar_ws")
async def lidar_ws(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            with device_lock:
                scan = lidar.read()
            await websocket.send_json(scan.model_dump())
            await asyncio.sleep(0.05)
    except WebSocketDisconnect:
        print("client disconnected")
    except Exception as e:
        print(e)
    finally:
        await websocket.close()


@router.get("/lidar", response_model=LidarScan)
def get_lidar():
    """
    returns the most current lidar scan
    """
    with device_lock:
        return lidar.read()
