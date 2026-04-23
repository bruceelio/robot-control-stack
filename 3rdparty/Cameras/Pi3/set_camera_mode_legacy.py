# 3rdparty/cameras/Pi3/set_camera_mode.py

from picamera2 import Picamera2
import time

WIDTH = 640
HEIGHT = 480
FPS = 30


def get_full_fov_res(model):
    """Returns the optimal binned full-FOV resolution based on sensor model."""
    modes = {
        "ov5647": (1296, 972),  # Camera Module v1
        "imx219": (1640, 1232),  # Camera Module v2
        "imx708": (2304, 1296),  # Camera Module v3 (16:9)
        "imx477": (2028, 1520),  # HQ Camera
    }
    # Default to v2 if unknown, or return the model's specific max binned mode
    return modes.get(model.lower(), (1640, 1232))


def main():
    picam2 = Picamera2()

    # 1. Detect the camera model
    camera_model = picam2.camera_details[0]['model']
    raw_res = get_full_fov_res(camera_model)

    print(f"Detected Camera: {camera_model}")
    print(f"Using Raw Resolution for Full FOV: {raw_res}")

    frame_duration = int(1_000_000 / FPS)

    # 2. Configure with the dynamic 'raw' size
    config = picam2.create_preview_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "RGB888"},
        raw={"size": raw_res},
        controls={
            "FrameDurationLimits": (frame_duration, frame_duration)
        }
    )

    picam2.configure(config)
    picam2.start()

    print(f"Camera configured to {WIDTH}x{HEIGHT} @ {FPS} FPS")
    time.sleep(1)

    # Confirm metadata
    metadata = picam2.capture_metadata()
    print("\n=== Camera Mode Confirmation ===")
    print("Actual Sensor Mode:", metadata.get("SensorSize"))  # Verify this size matches raw_res

    picam2.stop()


if __name__ == "__main__":
    main()
