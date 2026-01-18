# hw_io/cameras/sr_april.py

from hw_io.cameras.base import Camera

class SRAprilCamera(Camera):
    def __init__(self, sr_camera):
        self._cam = sr_camera

    def see(self):
        return self._cam.see()

    def capture(self):
        if hasattr(self._cam, "capture"):
            return self._cam.capture()
        raise NotImplementedError("Capture not supported")
