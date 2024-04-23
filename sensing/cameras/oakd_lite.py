"""Driver for the OAKD-Lite camera"""

import depthai as dai
import numpy as np
import cv2
from threading import Thread, Lock
import open3d as o3d
from time import sleep, time

class FPSTracker:
    """Class to track the FPS of a process. FPS is calculated as the average FPS over the last N_FRAMES frames."""
    N_FRAMES = 30
    def __init__(self):
        self._frame_count = 0
        self._start_time = None
        self._fps = 0
        self._fps_lock = Lock()

    @property
    def fps(self):
        with self._fps_lock:
            return self._fps
        
    def register_frame(self):
        self._frame_count += 1
        if self._start_time is None:
            self._start_time = time()
        if self._frame_count == self.N_FRAMES:
            end_time = time()
            with self._fps_lock:
                self._fps = self.N_FRAMES/(end_time - self._start_time)
            self._frame_count = 0
            self._start_time = end_time
class OAKDCam:
    """
    Driver class for camera. Uses a thread to continiously get images from camera.
    Images and pointclouds can be accessed using get_img() and get_pcd() methods.
    """
    def __init__(self):
        self.pipeline = dai.Pipeline()
        self._setup_color_cam()
        self._setup_mono_cams()
        self._setup_stereo_depth()
        self._setup_point_cloud()
        self._link_cameras()
        self._sync_cameras()
        self._setup_cam_output()

        self._img_lock = Lock()
        self._pcd_lock = Lock()
        self._running_status_lock = Lock()

        self._img = None
        self._pcd = None
        self._running = False
        self._fps_tracker = FPSTracker()
    
    def _setup_color_cam(self):
        self.color_cam = self.pipeline.create(dai.node.ColorCamera)
        self.color_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        self.color_cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        self.color_cam.setIspScale(1,5) # this helps with the frame rate, but it does reduce the RGB resolution

    def _setup_mono_cams(self):
        """"Set up the mono cameras the are used to compose the depth image"""
        self.mono_left_cam = self.pipeline.create(dai.node.MonoCamera)
        self.mono_left_cam.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.mono_left_cam.setCamera("left")

        self.mono_right_cam = self.pipeline.create(dai.node.MonoCamera)
        self.mono_right_cam.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.mono_right_cam.setCamera("right")

    def _setup_stereo_depth(self):
        """Set up the depth image feed"""
        self.stereo_depth_cam = self.pipeline.create(dai.node.StereoDepth)
        self.stereo_depth_cam.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        self.stereo_depth_cam.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        self.stereo_depth_cam.setLeftRightCheck(True)
        self.stereo_depth_cam.setExtendedDisparity(False)
        self.stereo_depth_cam.setSubpixel(True)
        self.stereo_depth_cam.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        config = self.stereo_depth_cam.initialConfig.get()
        config.postProcessing.thresholdFilter.minRange = 200
        config.postProcessing.thresholdFilter.maxRange = 3000
        self.stereo_depth_cam.initialConfig.set(config)

    def _setup_point_cloud(self):
        """Set up the point cloud feed"""
        self.point_cloud = self.pipeline.create(dai.node.PointCloud)
        self.point_cloud.initialConfig.setSparse(False)

    def _link_cameras(self):
        """Link the cameras together to get the depth image and point cloud"""
        # both mono cameras are used to get a depth image
        self.mono_left_cam.out.link(self.stereo_depth_cam.left)
        self.mono_right_cam.out.link(self.stereo_depth_cam.right)
        # depth image is used to get the pointcloud
        self.stereo_depth_cam.depth.link(self.point_cloud.inputDepth)

    def _sync_cameras(self):
        """Syncs the camera and point cloud feeds together"""
        self.sync = self.pipeline.create(dai.node.Sync)
        self.color_cam.isp.link(self.sync.inputs["rgb"])
        self.point_cloud.outputPointCloud.link(self.sync.inputs["pcl"])

    def _setup_cam_output(self):
        """Set up the output stream for the camera feed"""
        self.xOut = self.pipeline.create(dai.node.XLinkOut)
        self.xOut.input.setBlocking(False)
        self.xOut.setStreamName("out")
        self.sync.out.link(self.xOut.input)

    
    def _run(self):
        """Background thread that continiously gets images from the camera and updates the internal state of the class."""
        with dai.Device(self.pipeline) as device:
            q = device.getOutputQueue(name="out", maxSize=1, blocking=False)
            while True:
                with self._running_status_lock:
                    if not self._running:
                        break
                inMessage = q.get()
                inColor = inMessage["rgb"]
                inPointCloud = inMessage["pcl"]

                with self._img_lock:
                    self._img = inColor.getCvFrame()
                
                if inPointCloud:
                    with self._pcd_lock:
                        self._pcd = inPointCloud.getPoints().astype(np.float64)
                self._fps_tracker.register_frame()
    
    def start(self):
        """Start streaming images from the camera"""
        with self._running_status_lock:
            self._running = True
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()
        # The frame rate is low when the camera is started. This is a workaround to wait until the frame rate is high enough
        while True:
            if self.get_fps() > 10:
                break
            sleep(0.5)
    
    def stop(self):
        """Stop Streaming images from the camera"""
        with self._running_status_lock:
            self._running = False


    def get_img(self)->np.ndarray:
        """Get the latest image from the camera"""
        with self._img_lock:
            return self._img.copy()
        
    def get_jpg(self):
        frame = self.get_img()
        # encode to jpg
        ret, jpg = cv2.imencode(".jpg", frame)
        if not ret:
            raise Exception("Could not encode frame as jpg")
        return jpg.tobytes()
    
    def get_pcd(self)->o3d.geometry.PointCloud:
        """Get the latest point cloud from the camera"""
        with self._pcd_lock:
            points = self._pcd
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        return pcd
    
    def get_pcd_points(self):
        with self._pcd_lock:
            points = self._pcd
        return points
    
    def get_fps(self)->float:
        """Get the FPS of the camera feed"""
        return self._fps_tracker.fps
    
if __name__ == "__main__":

    cam = OAKDCam()
    cam.start()
    print("Started camera (Ctr+C to stop)")

    
    try:
        while True:
            latest_img = cam.get_img()
            latest_pcd = cam.get_pcd()
            print(f"FPS: {cam.get_fps()}")
            sleep(2)
    except KeyboardInterrupt:
        print("Stopping camera")
    finally:
        cam.stop()
    save = input("save latest frames? [y/N]").lower() == "y"
    if save:
        cv2.imwrite("frame.jpg", latest_img)
        o3d.io.write_point_cloud("frame.pcd", latest_pcd)
