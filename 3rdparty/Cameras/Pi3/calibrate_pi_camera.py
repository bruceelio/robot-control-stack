# 3rdparty/cameras/Pi3/calibrate_pi_camera.py
from __future__ import annotations

import argparse
import glob
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from picamera2 import Picamera2


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture chessboard images and calibrate a Pi camera"
    )

    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)

    parser.add_argument(
        "--pattern-cols",
        type=int,
        default=9,
        help="Number of inner corners across the chessboard",
    )
    parser.add_argument(
        "--pattern-rows",
        type=int,
        default=6,
        help="Number of inner corners down the chessboard",
    )
    parser.add_argument(
        "--square-size-mm",
        type=float,
        default=25.0,
        help="Physical chessboard square size in mm",
    )

    parser.add_argument(
        "--images-dir",
        type=str,
        default="calibration_images",
        help="Directory to store/read calibration images",
    )
    parser.add_argument(
        "--capture",
        action="store_true",
        help="Capture images interactively from the Pi camera",
    )
    parser.add_argument(
        "--solve",
        action="store_true",
        help="Solve calibration from saved images",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Show live camera preview during capture",
    )
    parser.add_argument(
        "--required-images",
        type=int,
        default=15,
        help="Suggested minimum number of good images",
    )

    return parser


def make_camera(width: int, height: int, fps: int) -> Picamera2:
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (width, height), "format": "RGB888"},
        controls={"FrameRate": fps},
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1.0)
    return picam2


def ensure_dir(path: str) -> Path:
    p = Path(path)
    p.mkdir(parents=True, exist_ok=True)
    return p


def capture_images(args: argparse.Namespace) -> int:
    images_dir = ensure_dir(args.images_dir)
    picam2 = make_camera(args.width, args.height, args.fps)

    pattern_size = (args.pattern_cols, args.pattern_rows)

    print("\n=== PI CAMERA CALIBRATION CAPTURE ===")
    print("Keys:")
    print("  c = capture image if chessboard is detected")
    print("  q = quit capture")
    print("")
    print(
        f"Target pattern: {args.pattern_cols}x{args.pattern_rows} inner corners, "
        f"square={args.square_size_mm} mm"
    )

    count = 0

    try:
        while True:
            frame_rgb = picam2.capture_array()
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)

            found, corners = cv2.findChessboardCorners(gray, pattern_size, None)

            display = frame_bgr.copy()
            if found:
                cv2.drawChessboardCorners(display, pattern_size, corners, found)
                cv2.putText(
                    display,
                    "Chessboard detected - press 'c' to capture",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
            else:
                cv2.putText(
                    display,
                    "Chessboard not detected",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

            cv2.putText(
                display,
                f"Saved images: {count}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 0, 0),
                2,
                cv2.LINE_AA,
            )

            cv2.imshow("Pi Camera Calibration Capture", display)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

            if key == ord("c"):
                if not found:
                    print("[CAPTURE] Chessboard not detected; image not saved")
                    continue

                filename = images_dir / f"calib_{count:03d}.png"
                cv2.imwrite(str(filename), frame_bgr)
                count += 1
                print(f"[CAPTURE] Saved {filename}")

    finally:
        picam2.stop()
        cv2.destroyAllWindows()

    print(f"\nCaptured {count} images")
    if count < args.required_images:
        print(
            f"[WARN] Fewer than {args.required_images} images captured; calibration may be weak"
        )

    return 0


def solve_calibration(args: argparse.Namespace) -> int:
    pattern_size = (args.pattern_cols, args.pattern_rows)
    square_size_m = args.square_size_mm / 1000.0

    images = sorted(glob.glob(os.path.join(args.images_dir, "*.png")))
    if not images:
        print(f"[ERROR] No images found in {args.images_dir!r}")
        return 1

    objp = np.zeros((args.pattern_cols * args.pattern_rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.pattern_cols, 0:args.pattern_rows].T.reshape(-1, 2)
    objp *= square_size_m

    objpoints: list[np.ndarray] = []
    imgpoints: list[np.ndarray] = []

    image_size = None
    used = 0

    print("\n=== SOLVING CAMERA CALIBRATION ===")
    print(f"Images found: {len(images)}")

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print(f"[WARN] Could not read {fname}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        image_size = gray.shape[::-1]

        found, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        if not found:
            print(f"[SKIP] No chessboard found in {fname}")
            continue

        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )
        corners2 = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            criteria,
        )

        objpoints.append(objp)
        imgpoints.append(corners2)
        used += 1
        print(f"[OK] Using {fname}")

    if used < 5:
        print("[ERROR] Need at least 5 good images to calibrate")
        return 1

    if image_size is None:
        print("[ERROR] Could not determine image size")
        return 1

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints,
        imgpoints,
        image_size,
        None,
        None,
    )

    fx = float(camera_matrix[0, 0])
    fy = float(camera_matrix[1, 1])
    cx = float(camera_matrix[0, 2])
    cy = float(camera_matrix[1, 2])

    total_error = 0.0
    for i in range(len(objpoints)):
        reprojected, _ = cv2.projectPoints(
            objpoints[i],
            rvecs[i],
            tvecs[i],
            camera_matrix,
            dist_coeffs,
        )
        error = cv2.norm(imgpoints[i], reprojected, cv2.NORM_L2) / len(reprojected)
        total_error += error

    mean_error = total_error / len(objpoints)

    print("\n=== CALIBRATION RESULT ===")
    print(f"Used images: {used}")
    print(f"Image size: {image_size}")
    print(f"RMS reprojection error: {ret}")
    print(f"Mean reprojection error: {mean_error}")
    print("")
    print(f"fx = {fx}")
    print(f"fy = {fy}")
    print(f"cx = {cx}")
    print(f"cy = {cy}")
    print("")
    print("Copy this into calibration/cameras/pi3_640_480.py:")
    print("")
    print(f"CAMERA_PARAMS = ({fx:.3f}, {fy:.3f}, {cx:.3f}, {cy:.3f})")
    print(f"DIST_COEFFS = {dist_coeffs.flatten().tolist()}")

    return 0


def main() -> int:
    args = build_parser().parse_args()

    if not args.capture and not args.solve:
        print("[ERROR] Choose at least one of --capture or --solve")
        return 1

    if args.capture:
        rc = capture_images(args)
        if rc != 0:
            return rc

    if args.solve:
        return solve_calibration(args)

    return 0


if __name__ == "__main__":
    sys.exit(main())