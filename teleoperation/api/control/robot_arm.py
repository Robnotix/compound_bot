from .router import router
from pydantic import BaseModel
from typing import Optional
from drivers.robot_control.mycobot_driver import se3_to_xyz_rpy, xyz_rpy_to_se3
from fastapi import WebSocket, Response, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from drivers.driver_dispatch import DriverDispatch


class Pose(BaseModel):
    x:float = 0.0
    y:float = 0.0
    z:float = 0.0
    rx:float = 0.0
    ry:float = 0.0
    rz:float = 0.0

    def __str__(self):
        return f"x={self.x}, y={self.y}, z={self.z}, rx={self.rx}, ry={self.ry}, rz={self.rz}"
    
    @classmethod
    def from_se3(cls,se3_mat):
        x,y,z,r,p,yaw = se3_to_xyz_rpy(se3_mat)
        return cls(x=x, y=y, z=z, rx=r, ry=p, rz=yaw)
    
    def to_se3(self):
        return xyz_rpy_to_se3(self.x, self.y, self.z, self.rx, self.ry, self.rz)

class RobotJoints(BaseModel):
    j1:Optional[float] = None
    j2:Optional[float] = None
    j3:Optional[float] = None
    j4:Optional[float] = None
    j5:Optional[float] = None
    j6:Optional[float] = None

    def __str__(self):
        return f"j1={self.j1}, j2={self.j2}, j3={self.j3}, j4={self.j4}, j5={self.j5}, j6={self.j6}"

@router.get("/robot_arm/speed")
def get_speed():
    with DriverDispatch.borrow("robot_arm") as robot:
        speed = robot.speed
    return speed

@router.put("/robot_arm/speed")
def set_speed(speed: int):
    with DriverDispatch.borrow("robot_arm") as robot:
        robot.speed = speed
    return {"message": f"Robot arm speed set to {speed}"}

@router.get("/robot_arm/pose", response_model=Pose)
def get_pose():
    """
    Returns the current pose of the robot arm
    """
    with DriverDispatch.borrow("robot_arm") as robot:
        pose_mat = robot.pose()
    return Pose.from_se3(pose_mat)


@router.post("/robot_arm/pose")
def set_pose(offset: Pose):
    """
    Moves the robot arm to the specified target position
    """
    with DriverDispatch.borrow("robot_arm") as robot:
        target_pose = offset.to_se3()
        robot.movel(target_pose)

@router.post("/robot_arm/reference")
def set_reference(pose:Pose):
    """
    Sets the specified pose relative to the robot base as the reference position
    """
    with DriverDispatch.borrow("robot_arm") as robot:
        robot.pose_offset = pose.to_se3()
    return {"message": f"Robot arm reference position set to {pose}"}

@router.get("/robot_arm/reference", response_model=Pose)
def get_reference():
    """
    Returns the current reference position of the robot arm
    """
    with DriverDispatch.borrow("robot_arm") as robot:
        reference_pose = robot.pose_offset
    return Pose.from_se3(reference_pose)

@router.post("/robot_arm/joints")
def set_joints(joints: RobotJoints):
    """
    Moves the robot arm to the specified joint positions
    """
    with DriverDispatch.borrow("robot_arm") as robot:
        joint_angles = [joints.j1, joints.j2, joints.j3, joints.j4, joints.j5, joints.j6]
        robot.movej(joint_angles)

@router.get("/robot_arm/joints")
def get_joints():
    """
    Returns the current joint positions of the robot arm
    """
    with DriverDispatch.borrow("robot_arm") as robot:
        joint_angles = robot.joints()
    return RobotJoints(j1=joint_angles[0], j2=joint_angles[1], j3=joint_angles[2], j4=joint_angles[3], j5=joint_angles[4], j6=joint_angles[5])

@router.websocket("/robot_arm/joints_ws")
async def joints_ws(websocket: WebSocket):
    """websocket to control the joints in real time"""
    await websocket.accept()
    try:
        while True:
            # receive data from client
            data = await websocket.receive_json()
            joints = RobotJoints.model_validate(data)
            with DriverDispatch.borrow("robot_arm") as robot:
                joint_angles = [joints.j1, joints.j2, joints.j3, joints.j4, joints.j5, joints.j6]
                robot.movej(joint_angles,blocking=False)
    except WebSocketDisconnect:
        print("client disconnected")
    except Exception as e:
        print(e)
    finally:
        await websocket.close()

# HTML endpoint with 6 sliders to control the robot joints and some js code to send the data to the websocket


@router.get("/robot_arm/joints_control_ui")
def joints_control():
    with robot_lock:
        joints = robot.joints()
    
    html = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Robot Joints Control</title>
    </head>
    <body>
        <h1>Robot Joints Control</h1>
        <div>
            <label for="j1">Joint 1</label>
            <input type="range" min="-3.14" max="3.14" value="{J1}" class="slider" id="j1">
        </div>
        <div>
            <label for="j2">Joint 2</label>
            <input type="range" min="-3.14" max="3.14" value="{J2}" class="slider" id="j2">
        </div>
        <div>
            <label for="j3">Joint 3</label>
            <input type="range" min="-3.14" max="3.14" value="{J3}" class="slider" id="j3">
        </div>
        <div>
            <label for="j4">Joint 4</label>
            <input type="range" min="-3.14" max="3.14" value="{J4}" class="slider" id="j4">
        </div>
        <div>
            <label for="j5">Joint 5</label>
            <input type="range" min="-3.14" max="3.14" value="{J5}" class="slider" id="j5">
        </div>
        <div>
            <label for="j6">Joint 6</label>
            <input type="range" min="-3.14" max="3.14" value="{J6}" class="slider" id="j6">
        </div>
        <script>
            var ws = new WebSocket("ws://er:8000/control/robot_arm/joints_ws");
            ws.onopen = function(event) {
                console.log("Connection open");
            };
            ws.onmessage = function(event) {
                console.log(event.data);
            };
            ws.onclose = function(event) {
                console.log("Connection closed");
            };
            var sliders = document.getElementsByClassName("slider");
            for (var i = 0; i < sliders.length; i++) {
                sliders[i].oninput = function() {
                    var data = {
                        j1: document.getElementById("j1").value,
                        j2: document.getElementById("j2").value,
                        j3: document.getElementById("j3").value,
                        j4: document.getElementById("j4").value,
                        j5: document.getElementById("j5").value,
                        j6: document.getElementById("j6").value
                    };
                    ws.send(JSON.stringify(data));
                }
            }
        </script>
    </body>
    """
    for i in range(6):
        html.replace(f"J{i+1}",str(joints[i]))

    return HTMLResponse(html)
