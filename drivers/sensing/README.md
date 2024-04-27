## Sensor Drivers

## Cameras

### Front facing
The MyAGV platform has a fron facing MPI camera. We interface with it using OpenCV
This module can be used stand-alone to make sure the drive is working:

"""
usage: front_facing.py [-h] [--camera CAMERA] [--save-as SAVE_AS] [--show]

optional arguments:
  -h, --help         show this help message and exit
  --camera CAMERA    Camera number. Example: 0 if camera is on /dev/video0
  --save-as SAVE_AS  Save frame as filename
  --show             Show frame in pop up window
"""

### OakD-Lite
There is a sensor mast attached to the deck of the MyAGV. The mast holds a OakD-Lite depth camera with a top down view of the robot.
This module allows us to stream RGB and pointcloud data from that camera.

## Lidar
The MyAGV robot has an onboard YDLidar X2 sensor. This module allows us to stream data from it.
"""
usage: lidar.py [-h] [--port PORT] [--duration DURATION] [--save-plot-as SAVE_PLOT_AS]

LidarX2 Driver

optional arguments:
  -h, --help            show this help message and exit
  --port PORT           Serial port
  --duration DURATION   Duration in seconds
  --save-plot-as SAVE_PLOT_AS
                        Save plot to file
"""
