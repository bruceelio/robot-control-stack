# config/__init__legacy.py

from . import __init__
from .schema import resolve

# -------------------------
# Load selected profile
# -------------------------
if run.PROFILE == "simulation":
    from config.profiles.simulation import *
elif run.PROFILE == "sr1":
    from .profiles.sr1 import *
elif run.PROFILE == "sr2":
    from .profiles.sr2 import *
else:
    raise ValueError(f"Unknown PROFILE '{run.PROFILE}'")

# -------------------------
# Resolve derived config
# -------------------------
_resolved = resolve(__init__, locals())

rotate_factor   = _resolved["rotate_factor"]
drive_factor    = _resolved["drive_factor"]
distance_scale  = _resolved["distance_scale"]
motor_polarity  = _resolved["motor_polarity"]

# -------------------------
# Optional logging
# -------------------------
if run.RUN_MODE.name != "NORMAL":
    print(
        "\n================ CONFIGURATION ================\n"
        f"Profile       : {run.PROFILE}\n"
        f"Robot ID      : {ROBOT_ID}\n"
        f"Environment   : {ENVIRONMENT}\n"
        f"Surface       : {SURFACE}\n"
        f"Drive Layout  : {DRIVE_LAYOUT}\n"
        f"Wheel Type    : {WHEEL_TYPE}\n"
        f"Rotate factor : {rotate_factor:.3f}\n"
        f"Drive factor  : {drive_factor:.3f}\n"
        f"Distance scale: {distance_scale:.3f}\n"
        f"Motor polarity: {motor_polarity}\n"
        "==============================================\n"
    )
