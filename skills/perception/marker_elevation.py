# skills/perception/marker_elevation.py

from __future__ import annotations

import math
from typing import Any


def _to_rad(v: float) -> tuple[float, str]:
    """
    Heuristic unit fix:
      - if magnitude looks like degrees (eg 4.5, 10, 20), convert to radians
      - if magnitude already small (<= ~1.3), assume radians
    """
    v = float(v)

    if abs(v) > 1.3:
        return math.radians(v), "deg->rad"

    return v, "rad"


def _get_dict_path(d: dict, *path: str) -> Any:
    cur: Any = d

    for k in path:
        if not isinstance(cur, dict) or k not in cur:
            return None

        cur = cur[k]

    return cur


def _marker_elevation(
    marker,
    *,
    img_h: int | None = None,
    fov_y_rad: float | None = None,
) -> tuple[float, str]:
    """
    Pose-free elevation cue.

    Priority order:
      1) position.vertical_angle
         (pose-free, from detector) [object or dict]

      2) orientation.pitch
         (pose-full-ish but sometimes available) [object or dict]

      3) pixel-centre/corners fallback
         (pose-free-ish)

    Returns:
        (pitch_rad, src_label)
    """

    # -----------------------------
    # 1) BEST: position.vertical_angle
    # -----------------------------

    pos = getattr(marker, "position", None)

    va = (
        getattr(pos, "vertical_angle", None)
        if pos is not None
        else None
    )

    if va is not None:
        try:
            pitch, unit = _to_rad(va)

            return (
                pitch,
                f"position.vertical_angle({unit})",
            )

        except (TypeError, ValueError):
            pass

    if isinstance(marker, dict):

        for key_path, label in (
            (
                ("position", "vertical_angle"),
                "marker['position']['vertical_angle']",
            ),
            (
                ("position_vertical_angle",),
                "marker['position_vertical_angle']",
            ),
            (
                ("vertical_angle",),
                "marker['vertical_angle']",
            ),
        ):

            va2 = _get_dict_path(
                marker,
                *key_path,
            )

            if va2 is not None:
                try:
                    pitch, unit = _to_rad(va2)

                    return (
                        pitch,
                        f"{label}({unit})",
                    )

                except (TypeError, ValueError):
                    pass

    # -----------------------------
    # 2) Next best: orientation.pitch
    # -----------------------------

    ori = getattr(marker, "orientation", None)

    op = (
        getattr(ori, "pitch", None)
        if ori is not None
        else None
    )

    if op is not None:
        try:
            pitch, unit = _to_rad(op)

            return (
                pitch,
                f"orientation.pitch({unit})",
            )

        except (TypeError, ValueError):
            pass

    if isinstance(marker, dict):

        for key_path, label in (
            (
                ("orientation", "pitch"),
                "marker['orientation']['pitch']",
            ),
            (
                ("pitch",),
                "marker['pitch']",
            ),
        ):

            op2 = _get_dict_path(
                marker,
                *key_path,
            )

            if op2 is not None:
                try:
                    pitch, unit = _to_rad(op2)

                    return (
                        pitch,
                        f"{label}({unit})",
                    )

                except (TypeError, ValueError):
                    pass

    # Pixel-based fallback needs camera geometry.
    if img_h is None or fov_y_rad is None:
        return 0.0, "none"

    # -----------------------------
    # 3) Fallback: image Y coordinate
    # -----------------------------

    for name in (
        "centre",
        "center",
        "centroid",
    ):

        c = getattr(marker, name, None)

        if c is None:
            continue

        if hasattr(c, "__len__") and len(c) >= 2:

            y_px = float(c[1])

            y_norm = max(
                0.0,
                min(
                    1.0,
                    y_px / float(img_h),
                ),
            )

            pitch = (
                0.5 - y_norm
            ) * fov_y_rad

            return (
                pitch,
                f"{name}(y)",
            )

        y = getattr(c, "y", None)

        if y is not None:

            y_px = float(y)

            y_norm = max(
                0.0,
                min(
                    1.0,
                    y_px / float(img_h),
                ),
            )

            pitch = (
                0.5 - y_norm
            ) * fov_y_rad

            return (
                pitch,
                f"{name}.y",
            )

    # -----------------------------
    # Corners fallback
    # -----------------------------

    corners = getattr(
        marker,
        "corners",
        None,
    )

    if (
        corners is not None
        and len(corners) >= 4
    ):

        ys = []

        for p in corners:

            if (
                hasattr(p, "__len__")
                and len(p) >= 2
            ):
                ys.append(
                    float(p[1])
                )

            else:
                py = getattr(
                    p,
                    "y",
                    None,
                )

                if py is not None:
                    ys.append(
                        float(py)
                    )

        if ys:

            y_px = sum(ys) / len(ys)

            y_norm = max(
                0.0,
                min(
                    1.0,
                    y_px / float(img_h),
                ),
            )

            pitch = (
                0.5 - y_norm
            ) * fov_y_rad

            return (
                pitch,
                "corners(avg_y)",
            )

    return 0.0, "none"