"""Candidate Pi Camera 3 configurations for repeatable testing.

Edit CONFIGS to add/remove candidate setups.

Notes:
- Keep width/height/fps fixed across calibration-related testing unless you intend
  to recalibrate for a different camera mode.
- Use fixed focus values once the focus workflow has selected a LensPosition.
- White balance can be locked with colour_gains if known; otherwise leave
  awb_lock_after_settle=True and record the chosen gains for later use.
"""

from __future__ import annotations

CONFIGS = [
    {
        "name": "baseline_640x480_30fps",
        "width": 640,
        "height": 480,
        "fps": 30,
        # 5000 us ~= 1/200 s
        "shutter_us": 5000,
        "analogue_gain": 2.0,
        # White balance options:
        # - Set awb_lock_after_settle=True to let AWB settle briefly, then lock it.
        # - Or set awb_lock_after_settle=False and provide colour_gains=(r, b).
        "awb_lock_after_settle": True,
        "awb_settle_s": 2.0,
        "colour_gains": None,
        # Focus options:
        # - fixed_lens_position=None means do not change focus in this runner.
        # - Set a numeric value after focus selection, for example 1.33.
        "fixed_lens_position": None,
        "notes": "Good starting point; lock AWB after settle.",
    },
    {
        "name": "shorter_shutter_lower_gain",
        "width": 640,
        "height": 480,
        "fps": 30,
        # 4000 us ~= 1/250 s
        "shutter_us": 4000,
        "analogue_gain": 2.0,
        "awb_lock_after_settle": True,
        "awb_settle_s": 2.0,
        "colour_gains": None,
        "fixed_lens_position": None,
        "notes": "Bias toward less motion blur.",
    },
    {
        "name": "longer_shutter_lower_noise",
        "width": 640,
        "height": 480,
        "fps": 30,
        # ~1/167 s
        "shutter_us": 6000,
        "analogue_gain": 1.5,
        "awb_lock_after_settle": True,
        "awb_settle_s": 2.0,
        "colour_gains": None,
        "fixed_lens_position": None,
        "notes": "Slightly brighter; test motion blur impact.",
    },
]
