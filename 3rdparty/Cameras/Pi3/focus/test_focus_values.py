#!/usr/bin/env python3
# 3rdparty/cameras/Pi3/focus/test_focus_values.py

from picamera2 import Picamera2, Preview
import argparse
import time
from pathlib import Path
import cv2

# Edit this list to test the values you want.
TEST_VALUES = [0.8, 1.0, 1.2]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Step through fixed focus values for manual AprilTag testing."
    )
    parser.add_argument(
        "--preview",
        choices=["drm", "none", "save"],
        default="none",
        help="Preview mode: drm = Pi monitor, none = no preview, save = save one frame per focus value",
    )
    parser.add_argument(
        "--save-dir",
        default="focus_debug",
        help="Directory for saved frames when --preview save",
    )
    parser.add_argument(
        "--settle-s",
        type=float,
        default=1.0,
        help="Seconds to wait after applying each LensPosition before reporting/saving",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    save_dir = Path(args.save_dir)
    if args.preview == "save":
        save_dir.mkdir(parents=True, exist_ok=True)

    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    )
    picam2.configure(config)

    if args.preview == "drm":
        picam2.start_preview(Preview.DRM)

    picam2.start()

    print("Camera started.")
    print(f"Preview mode: {args.preview}")
    print("For each focus value, test AprilTag detection and pose stability.")
    print("Press Enter to advance to the next value.")
    print("Type 'q' then Enter to quit early.")
    time.sleep(1.0)

    try:
        for i, value in enumerate(TEST_VALUES, start=1):
            print()
            print("=" * 72)
            print(f"[{i}/{len(TEST_VALUES)}] Applying LensPosition = {value}")

            picam2.set_controls({
                "AfMode": 0,          # Manual focus
                "LensPosition": value,
            })

            time.sleep(args.settle_s)

            metadata = picam2.capture_metadata()
            observed = metadata.get("LensPosition")
            af_mode = metadata.get("AfMode")
            af_state = metadata.get("AfState")

            print(f"Requested LensPosition: {value}")
            print(f"Observed LensPosition:  {observed}")
            print(f"AfMode: {af_mode}, AfState: {af_state}")
            print("Check:")
            print("- object-tag detection at 1–2 m")
            print("- stability during approach")
            print("- wall-tag usability")

            if args.preview == "save":
                frame = picam2.capture_array()
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                filename = save_dir / f"focus_{i:02d}_{value:.2f}.png"
                cv2.imwrite(str(filename), frame_bgr)
                print(f"[SAVE] Wrote {filename}")

            user_input = input("Press Enter for next value, or 'q' then Enter to quit: ").strip().lower()
            if user_input == "q":
                print("Stopping early at user request.")
                break

    finally:
        picam2.stop()
        print("Finished testing focus values.")


if __name__ == "__main__":
    main()