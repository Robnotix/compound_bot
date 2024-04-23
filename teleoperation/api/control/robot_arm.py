from .router import router
from pydantic import BaseModel
from typing import Optional

class Pose(BaseModel):
    x:float = 0.0
    y:float = 0.0
    z:float = 0.0
    roll:float = 0.0
    pitch:float = 0.0
    yaw:float = 0.0

    def __str__(self):
        return f"x={self.x}, y={self.y}, z={self.z}, roll={self.roll}, pitch={self.pitch}, yaw={self.yaw}"

class RobotJoints(BaseModel):
    j1:Optional[float] = None
    j2:Optional[float] = None
    j3:Optional[float] = None
    j4:Optional[float] = None
    j5:Optional[float] = None
    j6:Optional[float] = None

    def __str__(self):
        return f"j1={self.j1}, j2={self.j2}, j3={self.j3}, j4={self.j4}, j5={self.j5}, j6={self.j6}"

@router.post("/robot_arm/linear_move")
def linear_move(target: Pose):
    """
    Moves the robot arm to the specified target position in task space
    """
    return {"message": f"Robot arm moving to position {target}"}

@router.post("/robot_arm/linear_move_relative")
def linear_move_relative(offset: Pose):
    """
    Moves the robot arm to the specified target position relative from the current position
    """
    return {"message": f"Robot arm moving to relative position {offset}"}

@router.post("/robot_arm/set_reference")
def set_reference(pose:Pose):
    """
    Sets the specified pose relative to the robot base as the reference position
    """
    return {"message": f"Robot arm reference pose set to {pose}"}

@router.post("/robot_arm/joint_move")
def joint_move(joints: RobotJoints):
    """
    Moves the robot arm to the specified joint positions
    """
    return {"message": f"Robot arm moving to joint positions {joints}"}

