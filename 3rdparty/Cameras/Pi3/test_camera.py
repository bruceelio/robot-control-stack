# 3rdparty/cameras/Pi3/test_camera.py
from picamera2 import Picamera2, Preview
import argparse
import time
import cv2

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--preview", choices=["drm", "none", "save"], default="drm")
    args = parser.parse_args()

    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    )
    picam2.configure(config)

    if args.preview == "drm":
        picam2.start_preview(Preview.DRM)

    picam2.start()

    print(f"Running with preview mode: {args.preview}")

    try:
        while True:
            frame = picam2.capture_array()

            if args.preview == "save":
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.imwrite("test_frame.png", frame_bgr)
                print("Saved test_frame.png")
                time.sleep(2)

            else:
                time.sleep(1)

    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop()

if __name__ == "__main__":
    main()