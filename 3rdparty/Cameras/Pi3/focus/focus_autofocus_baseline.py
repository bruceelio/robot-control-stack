# 3rdparty/cameras/Pi3/focus/focus_autofocus_baseline.py

from picamera2 import Picamera2
import time


def wait_for_af_to_settle(picam2, timeout=5.0, poll_interval=0.2):
    """Wait briefly while autofocus settles and return latest metadata."""
    start = time.time()
    metadata = {}
    while time.time() - start < timeout:
        metadata = picam2.capture_metadata()
        af_state = metadata.get("AfState")
        lens_position = metadata.get("LensPosition")
        print(f"AF state: {af_state}, LensPosition: {lens_position}")
        time.sleep(poll_interval)
    return metadata


def main():
    picam2 = Picamera2()
    config = picam2.create_preview_configuration()
    picam2.configure(config)
    picam2.start()

    print("Camera started.")
    print("Point the camera at a realistic AprilTag test scene.")
    print("Include your normal object distance if possible.")
    time.sleep(2)

    print("Enabling continuous autofocus...")
    picam2.set_controls({"AfMode": 2})
    time.sleep(2)

    print("Switching to auto mode and triggering autofocus scan...")
    picam2.set_controls({"AfMode": 1})
    time.sleep(0.5)
    picam2.set_controls({"AfTrigger": 0})
    time.sleep(2)

    metadata = wait_for_af_to_settle(picam2, timeout=3.0)

    lens_position = metadata.get("LensPosition")
    af_state = metadata.get("AfState")

    print()
    print(f"Final AfState: {af_state}")
    print(f"Suggested baseline LensPosition: {lens_position}")

    if lens_position is None:
        print("ERROR: LensPosition was not reported.")
        print("Your camera/module may not support autofocus controls.")

    picam2.stop()
    print("Camera stopped.")


if __name__ == "__main__":
    main()
