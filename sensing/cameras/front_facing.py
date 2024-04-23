import cv2
import atexit
from threading import Thread, Lock
from time import sleep

DEFAULT_CAMERA = 0 # this sould correspond to a camera on /dev/video0

class FrontFacingCamera:
    def __init__(self, camera_number=DEFAULT_CAMERA):
        self.camera_number = camera_number

        self.cap = cv2.VideoCapture(self.camera_number)
        # set a small buffer size
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)

        self._frame = None
        self._frame_lock = Lock()

        self._should_run_thread = None
        self._should_run_thread_lock = Lock()

        if not self.cap.isOpened():
            raise Exception(f"Could not open camera {self.camera_number}")
    
        atexit.register(self._close)
    
    def read(self):
        with self._frame_lock:
            return self._frame.copy()
        
    def start(self):
        self._should_run_thread = True
        self._thread = Thread(target=self._run)
        self._thread.setDaemon(True)
        self._thread.start()

    def stop(self):
        with self._should_run_thread_lock:
            self._should_run_thread = False
        self._thread.join()
        
    def _run(self):
        while True:
            with self._should_run_thread_lock:
                if not self._should_run_thread:
                    break
            ret, frame = self.cap.read()
            if not ret:
                break
            with self._frame_lock:
                self._frame = frame
            sleep(0.025)
    
    def _close(self):
        self.cap.release()

    def save_frame(self, filename:str):
        frame = self.get_frame()
        cv2.imwrite(filename, frame)

    def get_jpg(self):
        frame = self.read()
        # encode to jpg str
        ret, jpg = cv2.imencode(".jpg", frame)
        if not ret:
            raise Exception("Could not encode frame as jpg")
        return jpg.tobytes()

if __name__ == "__main__":      
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=DEFAULT_CAMERA,help="Camera number. Example: 0 if camera is on /dev/video0")
    parser.add_argument("--save-as", type=str, default=None, help="Save frame as filename")
    parser.add_argument("--show", action="store_true", help="Show frame in pop up window")
    args = parser.parse_args()

    camera = FrontFacingCamera(camera_number=args.camera)
    frame = camera.get_frame()
    if args.save_as:
        camera.save_frame(args.save_as)
    if args.show:
        cv2.imshow("frame", frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()            
