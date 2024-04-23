from .router import router
from fastapi import WebSocket
import asyncio

# @router.get("/depth")
# async def get_color():
#     return {"color": "color"}


# @router.websocket("/depth_ws")
# async def websocket_endpoint(websocket: WebSocket):
#     await websocket.accept()
#     while True:
#         await websocket.send_text("here is your image")
#         await asyncio.sleep(1)
