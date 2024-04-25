"""Driver for the YDLidar X2"""

# A significant amout of the code was borrowed from https://github.com/nesnes/YDLidarX2_python

from serial import Serial
from time import sleep
from math import atan, pi
from threading import Thread, Lock
import atexit
import RPi.GPIO as GPIO
from pydantic import BaseModel
from typing import List, Tuple, Optional
import math


class LidarScan(BaseModel):
    measurements:List[Tuple[float,float]] = []

    def to_xy(self):
        x = []
        y = []
        for angle,distance in self.measurements:
            x.append(distance * math.cos(math.radians(angle)))
            y.append(distance * math.sin(math.radians(angle)))
        return x,y

    def plot(self,show=False, save_as:Optional[str]=None):
        import matplotlib.pyplot as plt
        x,y = self.to_xy()
        fig = plt.figure()
        plt.plot(x,y,'ro')
        if save_as:
            plt.savefig(save_as)
        if show:
            plt.show()
        return fig
    
    def __str__(self) -> str:
        min_angle = None
        max_angle = None
        min_distance = None
        max_distance = None
        for angle,distance in self.measurements:
            if min_angle is None or angle < min_angle:
                min_angle = angle
            if min_distance is None or distance < min_distance:
                min_distance = distance

            if max_angle is None or angle > max_angle:
                max_angle = angle
            if max_distance is None or distance > max_distance:
                max_distance = distance

        return f"LidarScan. angle_range = ({min_angle},{max_angle}). distance_range=({min_distance},{max_distance})"
        


class LidarX2:

    def __init__(self, port):
        self.port = port
        self.baudrate = 115200
        self.connected = False
        self.measureThread = None
        self.stopThread = False
        self._scan_data = LidarScan()
        self._scan_data_lock = Lock()
        self.serial = None
        self._open()

    def _turn_on(cls):
        """Turn on the lidar power"""
        GPIO.setmode(GPIO.BCM)
        sleep(0.01)
        GPIO.setup(20,GPIO.OUT)
        sleep(0.01)
        GPIO.output(20,1)
        sleep(0.5)

    def _turn_off(cls):
        """Turn power off"""
        GPIO.setmode(GPIO.BCM)
        sleep(0.01)
        GPIO.setup(20,GPIO.OUT)
        sleep(0.01)
        GPIO.output(20,0)

    def _open(self):
        try:
            self._turn_on()
            atexit.register(self._close)
            atexit.register(self._turn_off)
            if not self.connected:
                # Open serial
                self.serial = Serial(self.port, self.baudrate)
                timeout = 4000  # ms
                while not self.serial.isOpen() and timeout > 0:
                    timeout -= 10  # ms
                    sleep(0.01)  # 10ms
                if self.serial.isOpen():
                    self.connected = True
                    self.serial.flushInput()
                else:
                    return False
                # Start measure thread
                self.stopThread = False
                self.measureThread = Thread(target=LidarX2.__measureThread, args=(self,))
                self.measureThread.setDaemon(True)
                self.measureThread.start()
                return True
        except Exception as e:
            print(e)
        return False

    def _close(self):
        self.stopThread = True
        if self.measureThread:
            self.measureThread.join()
        if self.connected:
            self.serial.close()
            self.connected = False

    def read(self)->LidarScan:
        with self._scan_data_lock:
            return self._scan_data.model_copy()

    def __measureThread(self):
        startAngle = 0
        while not self.stopThread:
            measures = self.__readMeasures()
            if len(measures) == 0:
                continue
            # Get Start an End angles
            endAngle = measures[len(measures)-1][0]
            # Clear measures in the angle range
            with self._scan_data_lock:
                i = 0
                while i < len(self._scan_data.measurements):
                    angle = self._scan_data.measurements[i][0]
                    inRange = False
                    if endAngle > startAngle:
                        inRange = startAngle <= angle and angle <= endAngle
                    else:
                        inRange = (startAngle <= angle and angle <= 360) or (0 <= angle and angle <= endAngle)
                    if inRange:
                        self._scan_data.measurements.pop(i)
                        i -= 1
                    i += 1
                # Add measures
                for angle,distance in measures:
                    start_idx = 0
                    end_idx = len(self._scan_data.measurements) - 1
                    while start_idx <= end_idx:
                        mid_idx = (start_idx + end_idx) // 2
                        if self._scan_data.measurements[mid_idx][0] < angle:
                            start_idx = mid_idx + 1
                        else:
                            end_idx = mid_idx - 1
                    self._scan_data.measurements.insert(start_idx, (angle, distance))
            startAngle = endAngle

    def __readByte(self):
        # serial.read can return byte or str depending on python version...
        return self.__strOrByteToInt(self.serial.read(1))
    
    def __strOrByteToInt(self, value):
        if isinstance(value, str):
            return int(value.encode('hex'), 16)
        if isinstance(value, int):
            return value
        return int.from_bytes(value, byteorder='big')

    def __readMeasures(self):
        result = []
        # Check and flush serial
        if not self.connected:
            return result
        # Wait for data start bytes
        found = False
        checksum = 0x55AA
        while not found and not self.stopThread:
            while self.serial.read(1) != b"\xaa":
                pass
            if self.serial.read(1) == b"\x55":
                found = True
        if self.stopThread:
            return []
        # Check packet type
        ct = self.__readByte()
        if ct != 0:
            return result
        # Get sample count in packet
        ls = self.__readByte()
        sampleCount = ls#int(ls.encode('hex'), 16)
        if sampleCount == 0:
            return result
        # Get start angle
        fsaL = self.__readByte()
        fsaM = self.__readByte()
        fsa = fsaL + fsaM * 256
        checksum ^= fsa
        startAngle = (fsa>>1)/64
        # Get end angle
        lsaL = self.__readByte()
        lsaM = self.__readByte()
        lsa = lsaL + lsaM * 256
        endAngle = (lsa>>1)/64
        # Compute angle diff
        aDiff = float(endAngle - startAngle)
        if (aDiff < 0):
            aDiff = aDiff + 360
        # Get checksum
        csL = self.__readByte()
        csM = self.__readByte()
        cs = csL + csM * 256
        # Read and parse data
        dataRaw = self.serial.read(sampleCount*2)
        data = []
        for i in range(0, sampleCount*2):
            data.append(self.__strOrByteToInt(dataRaw[i]))
        for i in range(0, sampleCount*2, 2):
            # Get distance
            siL = data[i]
            siM = data[i+1]
            checksum ^= (siL + siM * 256)
            distance = float(siL + siM * 256)/4
            # Get angle and correct value from distance
            angle = startAngle+(aDiff/float(sampleCount))*i/2
            angleCorrection = 0
            if distance > 0:
                angleCorrection = (atan(21.8 * ((155.3 - distance) / (155.3 * distance))) * (180 / pi))
            angle = angle + angleCorrection
            if angle>360:
                angle = angle-360
            if angle<0:
                angle = angle+360
            # Append to result
            result.append((angle,distance))
        checksum ^= (ct + ls * 256)
        checksum ^= lsa
        # Validate checksum
        if checksum == cs:
            return result
        return []

if __name__ == "__main__":
    import time
    from argparse import ArgumentParser
    parser = ArgumentParser(description="LidarX2 Driver")
    parser.add_argument("--port", type=str, default="/dev/ttyAMA0", help="Serial port")
    parser.add_argument("--duration", type=int, default=20, help="Duration in seconds")
    parser.add_argument("--save-plot-as", type=str, default=None, help="Save plot to file")
    args = parser.parse_args()

    lidar = LidarX2(args.port)

    t = time.time()
    while time.time() - t < args.duration:  # Run for 20 seconds
        measures = lidar.read()  # Get latest lidar measures
        print(measures)
        time.sleep(0.5)

    if args.save_plot_as:
        measures.plot(save_as=args.save_plot_as, show=False)
