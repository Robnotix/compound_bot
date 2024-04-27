"""
This module provides a DriverDispatch class that manages the initialization and borrowing of various drivers.

The DriverDispatch class allows multiple components of the system to safely access and use different drivers 
concurrently by providing a context manager that ensures exclusive access to each driver.

The module also defines a dictionary, DRIVER_INIT_CONFIG, which contains the initial configuration for each driver. 
This configuration can be customized based on the specific requirements of the system.

Example usage:
    with DriverDispatch.borrow("lidar") as lidar_driver:
        # Use the lidar driver here

    with DriverDispatch.borrow("front_camera") as camera_driver:
        # Use the front-facing camera driver here

    # ...

"""

from threading import Lock
from contextlib import contextmanager
from typing import Dict, Any

from drivers.sensing.lidar import LidarX2
from drivers.sensing.cameras.front_facing import FrontFacingCamera
from drivers.sensing.cameras.oakd_lite import OAKDCam
from drivers.robot_control.mycobot_driver import MyCobot280
from drivers.robot_control.myagv_driver import AGV

DRIVER_INIT_CONFIG: Dict[str,Dict[str,Any]] = {
    "lidar": {
        "port": "/dev/ttyAMA0"
    },
    "front_camera": {
        "camera_number": 0
    },
    "depth_camera": {},
    "robot_arm": {
        "port": None # try to find port
        # "port": "/dev/ttyUSB0" # specific windows port
        # "port": "/dev/ttyACM0" # specific linux port
    },
}

class DriverDispatch:
    _drivers_classes: Dict[str, Any] = {
        "lidar": LidarX2,
        "front_camera": FrontFacingCamera,
        "depth_camera": OAKDCam,
        "robot_arm": MyCobot280,
        "agv": AGV
    }
    
    _drivers: Dict[str, Any] = {}
    _locks: Dict[str, Lock] = {}
    
    @classmethod
    @contextmanager
    def borrow(cls, driver_name: str):
        """
        Context manager that allows safe borrowing of a driver.

        Args:
            driver_name (str): The name of the driver to borrow.

        Yields:
            Any: The borrowed driver instance.

        Raises:
            AssertionError: If the specified driver is not found in the available drivers.

        Example usage:
            with DriverDispatch.borrow("lidar") as lidar_driver:
                # Use the lidar driver here
        """
        if driver_name not in cls._drivers:
            cls._create_driver(driver_name)
        try:
            cls._locks[driver_name].acquire()
            yield cls._drivers[driver_name]
        finally:
            cls._locks[driver_name].release()

    @classmethod
    def _create_driver(cls, driver_name: str)->None:
        """
        Creates a new instance of the specified driver.

        Args:
            driver_name (str): The name of the driver to create.

        Returns:
            Any: The created driver instance.

        Raises:
            AssertionError: If the specified driver is not found in the available drivers.
        """
        assert driver_name in cls._drivers_classes, f"Driver {driver_name} not found"
        print("Initiating driver: ", driver_name)
        driver_class = cls._drivers_classes[driver_name]
        config = DRIVER_INIT_CONFIG.get(driver_name, {})
        cls._locks[driver_name] = Lock()
        cls._drivers[driver_name] =  driver_class(**config)
    
