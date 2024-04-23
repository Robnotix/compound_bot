from fastapi import APIRouter
from .depth_cam import router as realsense_router

router = APIRouter()

router.include_router(realsense_router, prefix="/depth_camera")
