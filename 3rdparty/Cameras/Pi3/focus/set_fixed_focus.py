# 3rdparty/cameras/Pi3/focus/set_fixed_focus.py

from picamera2 import Picamera2
import time

# Replace this with your chosen tested value.
LENS_POSITION = 1.0     # 0.8 loses focus at 23 mm; 1.2 loses at 18mm (but too close anyway)


def main():
    picam2 = Picamera2()
    config = picam2.create_preview_configuration()
    picam2.configure(config)
    picam2.start()

    time.sleep(1)

    picam2.set_controls({
        "AfMode": 0,
        "LensPosition": LENS_POSITION,
    })

    print(f"Manual focus locked at LensPosition = {LENS_POSITION}")
    print("Keep this focus unchanged for calibration and runtime.")

    time.sleep(5)
    picam2.stop()


if __name__ == "__main__":
    main()
