from .router import router
from enum import Enum
from pydantic import BaseModel, field_validator
from typing import Optional

import asyncio
from fastapi import WebSocket, Response, WebSocketDisconnect
from fastapi.responses import HTMLResponse
import atexit

from drivers.driver_dispatch import DriverDispatch
from drivers.robot_control.myagv_driver import AGVState


@router.put("/agv/stop")
def stop_agv():
    """
    Sends a command to stop the AGV motion
    """
    with DriverDispatch.borrow("agv") as agv:
        agv.velocity = (0,0,0)
    return {"message": "AGV stopped"}

class MotionTypes(Enum):
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"
    CLOCKWISE = "clockwise"
    COUNTERCLOCKWISE = "counterclockwise"

class MotionCommand(BaseModel):
    motion: MotionTypes
    duration:Optional[float] = 1.0

    # add validator to make sure that duration is positive or None
    @field_validator('duration')
    @classmethod
    def validate_duration(cls, duration:Optional[float])->Optional[float]:
        if duration is not None and duration <= 0:
            raise ValueError('Duration must be positive or None')
        if duration > 5.0:
            raise ValueError('Duration must be less than 5 seconds')
        return duration

@router.post("/agv/move")
async def move_agv(command: MotionCommand):
    """
    Sends a command to move the AGV
    motion field can be one of the following:
    - forward
    - backward
    - left
    - right
    - clockwise
    - counterclockwise
    """
    speed = 0.5
    with DriverDispatch.borrow("agv") as agv:
        try:
            if command.motion == MotionTypes.FORWARD:
                agv.velocity = (speed, 0, 0)
            elif command.motion == MotionTypes.BACKWARD:
                agv.velocity = (-speed, 0, 0)
            elif command.motion == MotionTypes.LEFT:
                agv.velocity = (0, speed, 0)
            elif command.motion == MotionTypes.RIGHT:
                agv.velocity = (0, -speed, 0)
            elif command.motion == MotionTypes.CLOCKWISE:
                agv.velocity = (0, 0, -speed)
            elif command.motion == MotionTypes.COUNTERCLOCKWISE:
                agv.velocity = (0, 0, speed)
            await asyncio.sleep(command.duration)
        finally:
            agv.velocity = (0, 0, 0)
    return {"message": f"AGV moving {command.motion.value} for {command.duration} seconds"}

@router.get("/agv/state", response_model=AGVState)
def get_agv_state():
    """
    Returns the current state of the AGV
    """
    with DriverDispatch.borrow("agv") as agv:
        state = agv.state
    return state

class VelocityCommand(BaseModel):
    x : float
    y : float
    rz : float
    
@router.websocket("/agv/velocity_ws")
async def velocity_ws(websocket: WebSocket):
    """websocket to control the velocity in real time"""
    await websocket.accept()
    try:
        while True:
            # receive data from client
            data = await websocket.receive_json()
            command = VelocityCommand.model_validate(data)
            assert -1.0 <= command.x <= 1.0, "x velocity must be between -1.0 and 1.0"
            assert -1.0 <= command.y <= 1.0, "y velocity must be between -1.0 and 1.0"
            assert -1.0 <= command.rz <= 1.0, "rz velocity must be between -1.0 and 1.0"
            with DriverDispatch.borrow("agv") as agv:
                agv.velocity = (command.x, command.y, command.rz)
            await asyncio.sleep(0.05)
            await websocket.send_json({"message": "ok"})
    except WebSocketDisconnect:
        print("client disconnected")
    except Exception as e:
        print(e)
    finally:
        agv.velocity = (0.0, 0.0, 0.0)
        await websocket.close()


# HTML endpoint with 3 sliders and some js that sends the values to the websocket
html = """
<!DOCTYPE html>
<html>
    <head>
        <title>AGV Control</title>
        <script>
            var ws = new WebSocket("ws://er:8000/control/agv/velocity_ws");
            ws.onopen = function(event) {
                console.log("Connection open");
            };
            ws.onmessage = function(event) {
                console.log(event.data);
            };
            ws.onclose = function(event) {
                console.log("Connection closed");
            };

            function sendVelocity() {
                var x = document.getElementById("x").value;
                var y = document.getElementById("y").value;
                var rz = document.getElementById("rz").value;
                ws.send(JSON.stringify({x: parseFloat(x), y: parseFloat(y), rz: parseFloat(rz)}));
            }
        </script>
    </head>
    <body>
        <h1>AGV Control</h1>
        <p>Use the sliders to control the AGV velocity</p>
        <div>
            <label for="x">X velocity:</label>
            <input type="range" id="x" name="x" min="-1" max="1" step="0.01" value="0" oninput="sendVelocity()">
            <span id="x_value">0</span>
        </div>
        <div>
            <label for="y">Y velocity:</label>
            <input type="range" id="y" name="y" min="-1" max="1" step="0.01" value="0" oninput="sendVelocity()">
            <span id="y_value">0</span>
        </div>
        <div>
            <label for="rz">Rotation velocity:</label>
            <input type="range" id="rz" name="rz" min="-1" max="1" step="0.01" value="0" oninput="sendVelocity()">
            <span id="rz_value">0</span>
        </div>
    </body>
"""

@router.get("/agv/control_gui")
def agv_control():
    return HTMLResponse(html)
