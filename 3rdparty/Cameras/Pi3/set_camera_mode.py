# 3rdparty/cameras/Pi3/set_camera_mode.py

from picamera2 import Picamera2
import time

WIDTH = 640
HEIGHT = 480
FPS = 30

def main():
    picam2 = Picamera2()

    frame_duration = int(1_000_000 / FPS)  # microseconds

    config = picam2.create_preview_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "RGB888"},
        controls={
            "FrameDurationLimits": (frame_duration, frame_duration)
        }
    )

    picam2.configure(config)
    picam2.start()

    print(f"Camera configured to {WIDTH}x{HEIGHT} @ {FPS} FPS")

    # Let camera settle
    time.sleep(1)

    # Read back metadata to confirm
    metadata = picam2.capture_metadata()

    print("\n=== Camera Mode Confirmation ===")
    print("FrameDuration:", metadata.get("FrameDuration"))
    print("Expected (~us):", frame_duration)

    picam2.stop()

if __name__ == "__main__":
    main()