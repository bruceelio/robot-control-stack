# calibration/schema.py

from dataclasses import dataclass
from typing import Dict, Tuple


# --------------------------------------------------
# Camera calibration schema
# --------------------------------------------------


@dataclass(frozen=True)
class CameraOptical:
    """
    Optical / perception corrections.
    """
    distance_scale: float
    bearing_sign: float
    bearing_offset_deg: float


@dataclass(frozen=True)
class CameraMeta:
    """
    Non-functional metadata (safe for logging / UI).
    """
    resolution: Tuple[int, int]
    fov_deg: float
    description: str


@dataclass(frozen=True)
class CameraCalibration:
    optical: CameraOptical
    meta: CameraMeta


# --------------------------------------------------
# Robot calibration schema
# --------------------------------------------------

@dataclass(frozen=True)
class Calibration:
    # ----------------------------------------------
    # Drive calibration
    # ----------------------------------------------
    drive_switch_mm: float

    drive_power_short: float
    drive_power_long: float

    drive_m_short: float
    drive_b_short: float

    drive_m_long: float
    drive_b_long: float

    drive_velocity_curve: Tuple[Tuple[float, float], ...]

    # ----------------------------------------------
    # Rotation calibration (timed backend)
    # ----------------------------------------------
    rotate_switch_deg: float

    # Small-angle rotations
    rotate_power_small: float
    rotate_m_small: float
    rotate_b_small: float

    # Large-angle rotations
    rotate_power_large: float
    rotate_m_large: float
    rotate_b_large: float

    # Voltage compensation
    voltage_reference: float

    voltage_low_model: str
    voltage_low_a: float
    voltage_low_b: float | None

    voltage_high_model: str
    voltage_high_a: float
    voltage_high_b: float | None

    # ----------------------------------------------
    # Camera calibration
    # ----------------------------------------------
    cameras: Dict[str, CameraCalibration]

