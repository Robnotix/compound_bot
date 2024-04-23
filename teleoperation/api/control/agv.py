from .router import router
from enum import Enum
from pydantic import BaseModel, field_validator
from typing import Optional


@router.get("/agv/stop")
def stop_agv():
    """
    Sends a command to stop the AGV motion
    """
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
        return duration

@router.post("/agv/move")
def move_agv(command: MotionCommand):
    """
    Sends a command to move the AGV
    """
    return {"message": f"AGV moving {command.motion.value} for {command.duration} seconds"}


