# diagnostics/marker_pitches.py

import math
import time

from behaviors.init_escape import InitEscape
from perception import classify_markers, corrected_bearing_deg, corrected_distance
from level2.level2_canonical import Level2
from motion_backends import create_motion_backend
from config import CONFIG
from calibration.resolve import resolve as resolve_calibration
from hw_io.resolve import resolve_io
from localisation import Localisation


# -----------------------------
# Tunables
# -----------------------------
RUN_INIT_ESCAPE = True          # you said you'll likely run init_escape first; this makes it automatic
CAMERA_NAME = "front"
SAMPLES = 20                    # number of prints
PERIOD_S = 0.5                  # time between prints


def _deg(rad: float) -> float:
    return float(rad) * (180.0 / math.pi)


def _get_attr(obj, names):
    for n in names:
        if hasattr(obj, n):
            return getattr(obj, n)
    return None


def _pitch_sources(marker) -> list[tuple[str, float]]:
    """
    Return a list of (label, pitch_deg) from various possible fields.
    This is intentionally redundant so we can see what the sim provides.
    """
    out: list[tuple[str, float]] = []

    pos = getattr(marker, "position", None)
    if pos is not None:
        # Common SR fields:
        va = _get_attr(pos, ["vertical_angle", "verticalAngle"])
        if va is not None:
            try:
                out.append(("position.vertical_angle", _deg(float(va))))
            except Exception:
                pass

        # If we have a vector-like position with y/z (up/forward), compute elevation atan2(y,z)
        y = _get_attr(pos, ["y", "Y", "up", "Up"])
        z = _get_attr(pos, ["z", "Z", "forward", "Forward"])
        if y is not None and z is not None:
            try:
                y = float(y)
                z = float(z)
                if abs(z) > 1e-6:
                    out.append(("atan2(pos.y,pos.z)", _deg(math.atan2(y, z))))
            except Exception:
                pass

    # OpenCV-style translation vector
    tvec = getattr(marker, "tvec", None)
    if tvec is not None:
        try:
            if len(tvec) >= 3:
                y = float(tvec[1])
                z = float(tvec[2])
                if abs(z) > 1e-6:
                    out.append(("atan2(tvec[1],tvec[2])", _deg(math.atan2(y, z))))
        except Exception:
            pass

    # Orientation object (often present)
    ori = getattr(marker, "orientation", None)
    if ori is not None:
        p = _get_attr(ori, ["pitch"])
        if p is not None:
            try:
                # orientation.pitch is sometimes already degrees in SR stacks; sometimes radians.
                # We can't assume, so we print it raw as "orientation.pitch(raw)".
                out.append(("orientation.pitch(raw)", float(p)))
            except Exception:
                pass

    return out


def run(robot):
    print("\n=== MARKER PITCH DUMP DIAGNOSTIC (POSE-FREE) ===")

    # Core subsystems (same pattern as other diagnostics)
    io = resolve_io(robot=robot, hardware_profile=CONFIG.hardware_profile)
    lvl2 = Level2(io, max_power=CONFIG.max_motor_power)
    localisation = Localisation()  # NOT used for pose; only passed to InitEscape for compatibility

    calibration = resolve_calibration(config=CONFIG)

    motion_backend = create_motion_backend(
        CONFIG.motion_backend,
        lvl2,
        CONFIG,
        calibration,
    )

    # Optional: InitEscape to get to a place where markers are visible
    if RUN_INIT_ESCAPE:
        print("[DIAG] Running InitEscape...")
        behavior = InitEscape()
        behavior.start(config=CONFIG, motion_backend=motion_backend)

        while True:
            status = behavior.update(
                lvl2=lvl2,
                localisation=localisation,
                motion_backend=motion_backend,
            )
            if status.name == "SUCCEEDED":
                break
            time.sleep(0.02)

        print("[DIAG] InitEscape complete")
        time.sleep(CONFIG.camera_settle_time)

    cams = io.cameras()
    cam = cams.get(CAMERA_NAME)
    if cam is None:
        print(f"[DIAG] No camera named {CAMERA_NAME!r}. Available={list(cams.keys())}")
        return

    cam_cal = calibration.cameras[CAMERA_NAME]

    for i in range(SAMPLES):
        seen = cam.see() or []
        arena, acidic, basic = classify_markers(seen)

        rows = []
        for kind, ms in (("ARENA", arena), ("ACIDIC", acidic), ("BASIC", basic)):
            for m in ms:
                dist = corrected_distance(m, cam_cal)
                bearing = corrected_bearing_deg(m, cam_cal)

                pitches = _pitch_sources(m)
                if not pitches:
                    pitch_str = "none"
                else:
                    # show up to 2 sources to keep output readable
                    pitch_str = " | ".join([f"{lbl}={val:6.2f}" for (lbl, val) in pitches[:2]])

                rows.append((kind, int(m.id), float(dist), float(bearing), pitch_str))

        rows.sort(key=lambda r: (r[0], r[2]))  # kind, then distance

        print(f"\n--- sample {i+1}/{SAMPLES}   visible={len(seen)} (arena={len(arena)} acidic={len(acidic)} basic={len(basic)}) ---")
        if not rows:
            print("No markers visible")
        else:
            print("KIND    ID    dist_mm  bear_deg   pitch_sources")
            for kind, mid, dist, bear, pstr in rows:
                print(f"{kind:5s} {mid:5d} {dist:8.0f} {bear:9.2f}   {pstr}")

        time.sleep(PERIOD_S)

    print("\n=== END MARKER PITCH DUMP ===")
