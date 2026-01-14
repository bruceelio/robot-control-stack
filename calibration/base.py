# calibration/base.py
import numpy as np

# -----------------------------
# DRIVE CALIBRATION
# -----------------------------
DRIVE_POWER_SHORT = 0.6
DRIVE_POWER_LONG = 0.85
DRIVE_SWITCH_MM = 1000  # threshold

# Short distances (power = 0.6)
short_distances = np.array([250, 500, 750, 1000])
short_times     = np.array([0.34, 0.625, 0.85, 1.16])
m_short, b_short = np.polyfit(short_distances, short_times, 1)

# Long distances (power = 0.85)
long_distances = np.array([1000, 2000, 3000, 4000])
long_times     = np.array([0.835, 1.615, 2.4, 3.16])
m_long, b_long = np.polyfit(long_distances, long_times, 1)

def drive_duration(distance_mm):
    """Return (duration, power) for driving a distance_mm (mm)."""
    distance_mm = max(0, distance_mm)
    if distance_mm < DRIVE_SWITCH_MM:
        duration = m_short * distance_mm + b_short
        power = DRIVE_POWER_SHORT
    else:
        duration = m_long * distance_mm + b_long
        power = DRIVE_POWER_LONG
    return duration, power


# -----------------------------
# ROTATION CALIBRATION
# -----------------------------
ROTATE_POWER = 0.5

rotation_angles = np.array([90, 180, 270, 360])
rotation_times  = np.array([0.60, 0.91, 1.365, 1.85])
# rotation_times  = np.array([0.48, 0.91, 1.365, 1.85])
m_rot, b_rot = np.polyfit(rotation_angles, rotation_times, 1)


def rotate_duration(angle_deg):
    """Return (duration, power) for rotating angle_deg (deg)."""
    angle_deg = max(0, angle_deg)
    duration = m_rot * angle_deg + b_rot
    return duration, ROTATE_POWER


# -----------------------------
# Optional quick test
# -----------------------------
if __name__ == "__main__":
    print("\n-- Quick test --")
    for d in [100, 500, 1200, 2500]:
        dur, pwr = drive_duration(d)
        print(f"Drive {d} mm -> duration={dur:.2f}s, power={pwr}")

    for a in [45, 90, 180, 360]:
        dur, pwr = rotate_duration(a)
        print(f"Rotate {a} deg -> duration={dur:.2f}s, power={pwr}")
