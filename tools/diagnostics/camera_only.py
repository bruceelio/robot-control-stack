# diagnostics/camera_only.py
from __future__ import annotations

import math
import time

CAMERA_NAME = "front"
SAMPLES = 20
PERIOD_S = 0.5


def _deg(rad):
    if rad is None:
        return None
    return float(rad) * (180.0 / math.pi)


def _fmt(value, fmt: str) -> str:
    if value is None:
        return "None"
    return format(value, fmt)


def run(robot, io):
    print("\n=== CAMERA ONLY DIAGNOSTIC ===")

    cams = io.cameras()
    print(f"[DIAG] Available cameras: {list(cams.keys())}")

    cam = cams.get(CAMERA_NAME)
    if cam is None:
        print(f"[DIAG] No camera named {CAMERA_NAME!r}")
        return

    for i in range(SAMPLES):
        seen = cam.see() or []

        print(f"\n--- sample {i+1}/{SAMPLES} visible={len(seen)} ---")
        if not seen:
            print("No markers visible")
            time.sleep(PERIOD_S)
            continue

        print(
            "ID    size_m   dist_mm   bearing_deg   va_deg   "
            "cx_px   cy_px   yaw_deg   pitch_deg   roll_deg"
        )

        for m in seen:
            pos = getattr(m, "position", None)
            ori = getattr(m, "orientation", None)
            center = getattr(m, "center_px", None)

            dist = getattr(pos, "distance", None) if pos is not None else None
            bearing = getattr(pos, "horizontal_angle", None) if pos is not None else None
            va = getattr(pos, "vertical_angle", None) if pos is not None else None

            yaw = getattr(ori, "yaw", None) if ori is not None else None
            pitch = getattr(ori, "pitch", None) if ori is not None else None
            roll = getattr(ori, "roll", None) if ori is not None else None

            cx = center[0] if center is not None else None
            cy = center[1] if center is not None else None

            print(
                f"{int(getattr(m, 'id', -1)):3d}  "
                f"{_fmt(getattr(m, 'size', None), '.3f'):>7s}  "
                f"{_fmt(dist, '.0f'):>7s}  "
                f"{_fmt(_deg(bearing), '.2f'):>11s}  "
                f"{_fmt(_deg(va), '.2f'):>6s}  "
                f"{_fmt(cx, '.1f'):>6s}  "
                f"{_fmt(cy, '.1f'):>6s}  "
                f"{_fmt(_deg(yaw), '.2f'):>8s}  "
                f"{_fmt(_deg(pitch), '.2f'):>10s}  "
                f"{_fmt(_deg(roll), '.2f'):>9s}"
            )

        time.sleep(PERIOD_S)

    print("\n=== END CAMERA ONLY DIAGNOSTIC ===")