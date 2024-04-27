from .router import router
from fastapi import WebSocket, WebSocketDisconnect
import asyncio
from drivers.driver_dispatch import DriverDispatch
import numpy as np
from typing import List, Tuple
from fastapi.responses import FileResponse, HTMLResponse
import tempfile
import open3d as o3d
import json



@router.get("/point_cloud", response_model=List[Tuple[float,float,float]])
async def get_point_cloud():
    with DriverDispatch.borrow("depth_camera") as depth_camera:
        pcd = depth_camera.get_pcd()
    if pcd is not None:
        # pcd is an o3d PointCloud
        # downsample before retuning
        pcd = pcd.voxel_down_sample(voxel_size=5.0)
        # return list of points
        return np.asarray(pcd.points).tolist()
    
@router.get("/point_cloud.pcd",response_class=FileResponse)
def get_point_cloud_pcd():
    with DriverDispatch.borrow("depth_camera") as depth_camera:
        pcd = depth_camera.get_pcd()
    if pcd is not None:
        # pcd is an o3d PointCloud
        # downsample before retuning
        pcd = pcd.voxel_down_sample(voxel_size=5.0)
        # return .pcd file
        with tempfile.NamedTemporaryFile(delete=False, suffix=".pcd") as temp_file:
            temp_filename = temp_file.name
            o3d.io.write_point_cloud(temp_filename, pcd)
        return FileResponse(temp_filename)


@router.websocket("/point_cloud_ws")
async def point_cloud_ws(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            with DriverDispatch.borrow("depth_camera") as depth_camera:
                pcd = depth_camera.get_pcd()
            X,Y,Z = np.asarray(pcd.points).T
            data = {
                "x" : X.tolist(),
                "y" : Y.tolist(),
                "z" : Z.tolist()
            }
            await websocket.send_json(data)
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        print("client disconnected")
    finally:
        await websocket.close()



@router.get("/point_cloud_plot")
def get_point_cloud_plot():
    """serve html that renders a plot of the point cloud using plotly"""

    html = """
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <script src='https://cdn.plot.ly/plotly-2.31.1.min.js'></script>
        <script src='https://cdnjs.cloudflare.com/ajax/libs/d3/3.5.17/d3.min.js'></script>
    </head>

    <body>
        <div id='myDiv'></div>
        <script>
            var trace = {
                x: {X},
                y: {Y},
                z: {Z},
                mode: 'markers',
                marker: {
                    size: 1,
                    line: {
                    color: 'rgb(217, 217, 217)',
                    width: 0.5},
                },
                type: 'scatter3d'
            };

            var data = [trace];
            var layout = {
                margin: {
                    l: 0,
                    r: 0,
                    b: 0,
                    t: 0
                }
            };

            Plotly.react('myDiv', data, layout);

            // var ws = new WebSocket("ws://er:8000/sensors/depth_camera/point_cloud_ws");
            // ws.onmessage = function(event) {
            //     var data = JSON.parse(event.data);
            //     drawPointCloud(data);
            // };

            // function drawPointCloud(data){
            
            //     var trace = {
            //         x: data.x,
            //         y: data.y,
            //         z: data.z,
            //         mode: 'markers',
            //         marker: {
            //             size: 1,
            //             line: {
            //             color: 'rgb(217, 217, 217)',
            //             width: 0.5},
            //         },
            //         type: 'scatter3d'
            //     };
            //     var traces = [trace];
            //     var layout = {
            //         margin: {
            //             l: 0,
            //             r: 0,
            //             b: 0,
            //             t: 0
            //         }
            //     };
            //     Plotly.react('myDiv', traces);
            // };
        </script>
    </body>
    """
    with DriverDispatch.borrow("depth_camera") as depth_camera:
        pcd = depth_camera.get_pcd()
    if pcd is not None:
        # downsample before retuning
        pcd = pcd.voxel_down_sample(voxel_size=5.0)
        # return html
        X,Y,Z = np.asarray(pcd.points).T
        html = html.replace("{X}",json.dumps(X.tolist()))
        html = html.replace("{Y}",json.dumps(Y.tolist()))
        html = html.replace("{Z}",json.dumps(Z.tolist()))
        return HTMLResponse(html)
