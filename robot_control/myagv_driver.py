from pydantic import BaseModel, Field
from threading import Lock, Thread
from typing import Tuple
from time import monotonic, sleep
import atexit
from pymycobot.myagv import MyAgv


class BatteryStatus(BaseModel):
    charging:bool = None
    voltage:float = None
    inserted:bool = None
class BatteryCheck(BaseModel):
    time_checked:float = None
    battery_1:BatteryStatus = BatteryStatus()
    battery_2:BatteryStatus = BatteryStatus()

class AGVState(BaseModel):
    x_velocity: float = 0.0
    y_velocity: float = 0.0
    rz_velocity: float = 0.0
    # battery status is expresed as Typle[time_checked, battery_voltage]
    battery_status:BatteryCheck = BatteryCheck()

class AGV:
    def __init__(self,port:str="/dev/ttyAMA2"):
        self.agv = MyAgv(port=port)
        
        self._state = AGVState()
        self._state_lock = Lock()

        self._run_thread = Thread(target=self._run_thread,daemon=True)
        self._should_run = True
        self._run_thread.start()
        atexit.register(self.quit)
    
    # setters and getters for each state variable
    @property
    def velocity(self):
        with self._state_lock:
            return (self._state.x_velocity,self._state.y_velocity,self._state.rz_velocity)
    
    @velocity.setter
    def velocity(self,vel:Tuple[float,float,float]):
        x_velocity,y_velocity,rz_velocity = vel
        for val in (x_velocity,y_velocity,rz_velocity):
            if not -1.0 <= val <= 1.0:
                raise ValueError("Velocity must be between -1.0 and 1.0")
        with self._state_lock:
            self._state.x_velocity = x_velocity
            self._state.y_velocity = y_velocity
            self._state.rz_velocity = rz_velocity
        print(f"Set velocity to {(x_velocity,y_velocity,rz_velocity)}")

    @property
    def state(self):
        with self._state_lock:
            return self._state.model_copy()
    
    def quit(self):
        with self._state_lock:
            self._should_run = False
    
    def stop(self):
        """stop the AGV"""
        with self._state_lock:
            self._state.x_velocity = 0.0
            self._state.y_velocity = 0.0
            self._state.rz_velocity = 0.0

    def _map_vel(self,vel:float):
        """scale the speed values from -1 to 1 to 0-255"""
        return int(127*vel)+128
    
    def _run_thread(self):
        x_velocity,y_velocity,rz_velocity = 0.0,0.0,0.0
        while True:
            with self._state_lock:
                new_x_velocity,new_y_velocity,new_rz_velocity = self._state.x_velocity,self._state.y_velocity,self._state.rz_velocity
            if new_x_velocity != x_velocity or new_y_velocity != y_velocity or new_rz_velocity != rz_velocity:
                # scale values so that they are ints between 0-255
                x_velocity_cmd = self._map_vel(new_x_velocity)
                y_velocity_cmd = self._map_vel(new_y_velocity)
                rz_velocity_cmd = self._map_vel(new_rz_velocity)
                print(f"sending command {(x_velocity_cmd,y_velocity_cmd,rz_velocity_cmd)}")
                self.agv._mesg(x_velocity_cmd,y_velocity_cmd,rz_velocity_cmd)
                
                # update velocity values
                x_velocity,y_velocity,rz_velocity = new_x_velocity,new_y_velocity,new_rz_velocity

            # check the battery status every 30 seconds
            if self._state.battery_status.time_checked is None or monotonic()-self._state.battery_status.time_checked > 3.0:
                battery_info = self.agv.get_battery_info()
                if isinstance(battery_info,list) and len(battery_info) == 3:
                    battery_data, v1, v2 = battery_info
                    battery_data_bits =[bool(int(bit)) for bit in battery_data]
                    with self._state_lock:
                        self._state.battery_status.battery_1.voltage = v1
                        self._state.battery_status.battery_2.voltage = v2
                        self._state.battery_status.time_checked = monotonic()

                        self._state.battery_status.battery_1.charging = battery_data_bits[2]
                        self._state.battery_status.battery_2.charging = battery_data_bits[4]
                        self._state.battery_status.battery_1.inserted = battery_data_bits[1]
                        self._state.battery_status.battery_2.inserted = battery_data_bits[0]
            sleep(0.1)
    
