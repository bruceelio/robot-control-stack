# hw_io/cameras/async_camera_proxy.py

class AsyncCameraProxy:
    def __init__(self, camera_name, camera_manager):
        self.camera_name = camera_name
        self.camera_manager = camera_manager

    def see(self):
        message = self.camera_manager.get_latest(self.camera_name)

        if not message:
            return []

        markers = message.get("markers")

        if markers is None:
            return []

        return list(markers)