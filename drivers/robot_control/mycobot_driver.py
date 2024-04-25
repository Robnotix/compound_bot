from pymycobot.mycobot import MyCobot
import numpy as np
import numpy.typing as npt
from contextlib import contextmanager
import time
import atexit
from pybotics.robot import Robot
from pybotics.tool import Tool
from typing import Optional

# some handy functions for working with poses
def se3_to_xyz_rpy(se3_matrix):
    """
    Convert an SE3 matrix to translation (x, y, z) and rotation (rx, ry, rz) in radians.
    The rotation is represented as Tait-Bryan angles in the 'zyx' convention.
    """
    # Extract translation
    x, y, z = se3_matrix[0:3, 3]

    # Extract rotation matrix
    R = se3_matrix[0:3, 0:3]

    # Compute Euler angles
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        rx = np.arctan2(R[2, 1], R[2, 2])
        ry = np.arctan2(-R[2, 0], sy)
        rz = np.arctan2(R[1, 0], R[0, 0])
    else:
        rx = np.arctan2(-R[1, 2], R[1, 1])
        ry = np.arctan2(-R[2, 0], sy)
        rz = 0

    return x, y, z, rx, ry, rz

def xyz_rpy_to_se3(x, y, z, rx, ry, rz):
    """
    Convert translation (x, y, z) and rotation (rx, ry, rz) in radians to an SE3 matrix.
    The rotation is represented as Tait-Bryan angles in the 'zyx' convention.
    """
    # Compute rotation matrix
    Rz = np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz), np.cos(rz), 0],
        [0, 0, 1]
    ])

    Ry = np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)]
    ])

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx), np.cos(rx)]
    ])

    R = Rz @ Ry @ Rx

    # Create SE3 matrix
    se3_matrix = np.eye(4)
    se3_matrix[0:3, 0:3] = R
    se3_matrix[0:3, 3] = np.array([x, y, z])

    return se3_matrix

def validate_se3_pose(se3_pose):
    """check that the given SE3 pose is valid"""
    if se3_pose.shape != (4, 4):
        raise ValueError("SE3 pose must be a 4x4 matrix")

    if np.linalg.det(se3_pose[0:3, 0:3]) < 0:
        raise ValueError("Rotation matrix must have a positive determinant")

    if not np.allclose(se3_pose[3,::],np.array([0.0, 0.0, 0.0, 1.0])):
        raise ValueError("Bottom row of SE3 pose must be [0, 0, 0, 1]")

class MyCobot280:
    """
    Class for interfacing with the Elephant Robotics MyCobot280
    """
    # DH Parameters for the MyCobot280
    DH_PARAMS = np.array([
        [0.0, 0.0, 0.0, 131.56/1000],
        [np.pi/2, 0.0, -np.pi/2, 0.0],
        [0.0, -110.4/1000, 0, 0],
        [0.0, -96.0/1000, -np.pi/2, 64.62/1000],
        [np.pi/2, 0.0, np.pi/2, 73.18/1000 ],
        [-np.pi/2, 0.0, 0.0,48.6/1000]
    ])
    
    def __init__(self,port:Optional[str]=None,log_file=None):
        self._connect(port)
        self._setup_logging(log_file)
        self._setup_dh_robot()

        self._speed = None
        self._tx_base_offset = np.eye(4)

    def _connect(self,port:Optional[str]=None):
        """
        Try connecting to robot on specified port. Flash light pattern if successful.
        """
        self.cobot = None
        if port is None:
            # if port is not given, scan the COM ports to see if one works
            import serial.tools.list_ports
            ports = list(serial.tools.list_ports.comports())
            connected = False
            for port in ports:
                print(port.device)
                try:
                    self.cobot = MyCobot(port.device,115200)
                    assert self.cobot.is_controller_connected() == 1, "could not connect to the cobot"
                    print(f"Connected to robot on port {port.device}")
                    connected = True
                    break
                except:
                    pass
            if  not connected:
                raise Exception("Could not find a robot to connect to")
            
        else:
            self.cobot = MyCobot(port,115200)
            assert self.cobot.is_controller_connected() == 1, "could not connect to the cobot"
        self.cobot.power_on()
        # show a color pattern to show that the robot is connected
        self.cobot.set_color(255,0,0)
        time.sleep(0.25)
        self.cobot.set_color(0,255,0)
        time.sleep(0.25)
        self.cobot.set_color(0,0,255)
        time.sleep(0.25)
        
    def _setup_logging(self,log_file):
        """
        Set up a log file to record joint angles
        TODO: figure out a better format to log to (like mcap)
        """
        if log_file:
            self._log_file = open(log_file,"w")
            self._log_file.write("Time,J1,J2,J3,J4,J5,J6\n")
            atexit.register(self._log_file.close)
        else:
            self._log_file = None

    def _setup_dh_robot(self):
        """Set up a pybotics representation of the robot. This is used for forward/inverse kinematics calculations"""
        self._dh_robot = Robot.from_parameters(self.DH_PARAMS)
        self._dh_robot.tool = Tool()
        self._dh_robot.tool.matrix = np.eye(4)

    def _log_joints(self):
        """Log the joint values to file"""
        if self._log_file is not None:
            try:
                joints = self.joints()
                self._log_file.write(str(time.monotonic())+"," + ",".join(map(str,joints)) + "\n")
            except:
                print("could not log joint angles")

    def _monitor_move(self,timeout=10.0):
        '''Wait for the robot to finish moving. If it takes longer than timeout, raise an exception'''
        t_init = time.monotonic()
        while self.cobot.is_moving():
            if time.monotonic()-t_init > timeout:
                raise Exception("Timeout waiting for move to complete")
            self._log_joints()
            time.sleep(0.005)
            
    def joints(self)->npt.NDArray[np.float64]:
        """return joints in radians"""
        for i in range(3):
            angles = self.cobot.get_angles() # returns values in degrees
            if angles:
                angles = np.radians(angles)
                self._dh_robot.joints = angles
                return angles
            time.sleep(0.01)
        raise Exception("Could not get joint angles")

    def pose(self)->npt.NDArray[np.float64]:
        """return se3 (4x4 matrix) for the current pose of the robot"""
        return np.linalg.inv(self._tx_base_offset) @ self._dh_robot.fk(self.joints())


    def movej(self,joints:npt.NDArray[np.float64],blocking=True):
        """linear move in joint space"""
        if self._speed is None:
            raise Exception("Speed must be set before moving")
        self.cobot.send_angles(list(np.degrees(joints)),self._speed)
        if blocking:
            self._monitor_move()

    def movel(self,pose:npt.NDArray[np.float64],blocking=True):
        """linear move in se3 space"""
        # this currently uses the send_coords method of the MyCobot driver
        # send_coords does produce a very accurate linear trajectory
        # TODO: explore if calculating our own trajectory and streaming it to the robot yields better results
        # TODO: add a check to make sure the pose is reachable without encountering singularities along the way
        if self._speed is None:
            raise Exception("Speed must be set before moving")
        offset_pose = self._tx_base_offset @ pose
        coords = list(se3_to_xyz_rpy(offset_pose))
        # convert from m to mm
        for i in range(3):
            coords[i] *= 1000
        # convert rad to deg
        for i in range(3):
            coords[i+3] = np.degrees(coords[i+3])
        
        self.cobot.send_coords(coords,self._speed,mode=1)
        if blocking:
            self._monitor_move()
    
    @property
    def pose_offset(self):
        """
        set a pose offset relative to the robot base
        future movel commands will be relative to this pose
        """
        return self._tx_base_offset.copy()

    @pose_offset.setter
    def pose_offset(self,pose:npt.NDArray[np.float64]):
        validate_se3_pose(pose)
        self._tx_base_offset = pose


    @property
    def tool_pose(self):
        """set the pose of the tool center point (TCP) relative to the robot flange"""
        return self._dh_robot.tool.matrix

    @tool_pose.setter
    def tool_pose(self,pose:npt.NDArray[np.float64]):
        validate_se3_pose(pose)
        self._dh_robot.tool.matrix = pose

    @property
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self,value:int):
        """set speed, must be between 0 and 100"""
        assert value > 0 and value <= 100, "Speed must be between 0 and 100"
        self._speed = value

    @contextmanager
    def freedrive(self):
        """Temporarily release the motors to allow for freedriving"""
        try:
            self.cobot.release_all_servos()
            yield
        finally:
            self.cobot.power_on()

    def record_program(self,save_as:str):
        """Record a program of joint and linear moves to a file. Press Q to quit recording."""
        # TODO: add more commands (set_speed, set_reference_frame, etc...)
        input("Press ENTER to start recording. Motors will turn off, allowing you to freedrive robot")
        with self.freedrive():
            with open(save_as,"w") as f:
                while True:
                    usr_in = input("J for joint move, L for linea move, Q to quit")
                    if usr_in == "Q":
                        break
                    elif usr_in == "J":
                        joints = self.joints()
                        print(f"Recorded joints: {joints}")
                        command = "movej," + ",".join(map(str,joints))
                        f.write(command + "\n")
                    elif usr_in == "L":
                        pose = self.pose()
                        print(f"Recorded pose: {pose}")
                        command = "movel," + ",".join(map(str,pose))
                        f.write(command + "\n")
                    else:
                        print("Invalid input")
    
    def run_program(self,program:str):
        """Replay a saved program"""
        with open(program,"r") as f:
            for line in f:
                print(line)
                command = line.strip().split(",")
                if command[0] == "movej":
                    joints = list(map(float,command[1:]))
                    self.movej(joints)
                elif command[0] == "movel":
                    pose = list(map(float,command[1:]))
                    self.movel(pose)
                else:
                    print("Invalid command in program")
                    break
