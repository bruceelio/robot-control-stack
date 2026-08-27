# config/profiles/hw_sr2026.py

from .simulation_legacy import *  # noqa

ROBOT_ID = "sr1"
HARDWARE_PROFILE = "sr2026"
ENVIRONMENT = "real"
SURFACE = "tile"

CAMERAS = {
    "front": "sr",
}