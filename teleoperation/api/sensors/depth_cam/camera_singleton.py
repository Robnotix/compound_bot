from sensing.cameras.oakd_lite import OAKDCam
from threading import Lock
import atexit

depth_camera = OAKDCam()
depth_camera.start()

depth_cam_lock = Lock()

atexit.register(depth_camera.stop)
