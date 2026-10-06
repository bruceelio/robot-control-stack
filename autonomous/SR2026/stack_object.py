# autonomous/SR2026/stack_object.py

from __future__ import annotations

import math

from autonomous.SR2026.base import Behavior, BehaviorStatus
from primitives.base import PrimitiveStatus
from primitives.motion import Drive
from primitives.manipulation import Release
from skills.navigation.align_to_target import AlignToTarget


class StackObject(Behavior):
    """
    Final stacking sequence after perception-guided approach to a
    delivered LOW cube.

    The selected target is always the LOW cube forming the base of
    the stack.

    Geometry:
        - approach / handoff geometry is LOW-cube geometry
        - final commitment uses the same LOW-cube commitment used
          by pickup
        - lift height is the robot-profile stack height

    Sequence:
        stack lift
        -> final alignment
        -> LOW-cube final commitment
        -> release carried cube
        -> raise to retreat height
        -> reverse full commitment
        -> SUCCEEDED

    Lowering/resetting the lift remains owned by the existing
    post-dropoff behaviour for now.
    """

    FINAL_ALIGNMENT_TOLERANCE_DEG = 1.0

    def __init__(self):
        super().__init__()

        self.config = None

        self.target_id: int | None = None

        self.distance_mm = 0.0
        self.bearing_deg = 0.0
        self.final_drive_mm = 0.0

        self.object_released = False
        self._step = None

        self._align = None
        self._drive = None
        self._release = None
        self._retreat = None

    def start(
        self,
        *,
        config,
        lvl2,
        motion_backend,
        distance_mm: float,
        bearing_deg: float,
        target_id: int | None = None,
        **_,
    ):
        self.config = config
        self.status = BehaviorStatus.RUNNING

        self.target_id = (
            None
            if target_id is None
            else int(target_id)
        )

        self.distance_mm = float(
            distance_mm
        )

        self.bearing_deg = float(
            bearing_deg
        )

        if (
            not math.isfinite(self.distance_mm)
            or self.distance_mm <= 0.0
        ):
            raise ValueError(
                "stack handoff distance must be finite and > 0 mm"
            )

        if not math.isfinite(
            self.bearing_deg
        ):
            raise ValueError(
                "stack handoff bearing must be finite"
            )

        # This is deliberately LOW-cube commitment geometry.
        #
        # ApproachTargetServo has already stopped at the LOW
        # final_commit_distance_mm because stacking calls it with:
        #
        #     fixed_target_is_high=False
        #
        # The remaining blind commitment therefore follows the
        # same geometry/calibration as LOW pickup.
        push_mm = float(
            config.final_approach_marker_push
        )

        self.final_drive_mm = max(
            0.0,
            self.distance_mm + push_mm,
        )

        if (
            not math.isfinite(
                self.final_drive_mm
            )
            or self.final_drive_mm <= 0.0
        ):
            raise ValueError(
                "stack final commitment must be finite and > 0 mm"
            )

        self._align = None
        self._drive = None
        self._release = None
        self._retreat = None
        self.object_released = False

        print(
            "[STACK_OBJECT] start "
            f"target_id={self.target_id} "
            f"dist={self.distance_mm:.0f}mm "
            f"bearing={self.bearing_deg:+.1f}deg "
            f"commitment={self.final_drive_mm:.0f}mm"
        )

        # The approach reference was the LOW base cube, but the
        # carried cube must be physically positioned at stack height.
        if not self._command_lift(
            lvl2,
            "lift_stack_position",
            "STACK",
        ):
            return self.status

        self._align = AlignToTarget(
            bearing_deg=self.bearing_deg,
            tolerance_deg=(
                self.FINAL_ALIGNMENT_TOLERANCE_DEG
            ),
            max_rotate_deg=float(
                config.max_rotate_deg
            ),
        )

        self._align.start(
            motion_backend=motion_backend
        )

        self._step = "ALIGN"

        return self.status

    def _command_lift(
        self,
        lvl2,
        config_attr,
        label,
    ) -> bool:
        """
        Command a named robot-profile lift position through canonical IO.
        """

        try:
            position = float(
                getattr(
                    self.config,
                    config_attr,
                )
            )

            if (
                not math.isfinite(position)
                or not -1.0 <= position <= 1.0
            ):
                raise ValueError(
                    f"{config_attr} out of servo range: "
                    f"{position}"
                )

            io = lvl2.io

            io.servo["lift"].position = (
                position
            )

            # Reuse the existing manipulator settle calibration
            # until testing shows stacking needs its own value.
            settle_s = float(
                self.config.pickup_lift_settle_s
            )

            if (
                not math.isfinite(settle_s)
                or settle_s < 0.0
            ):
                raise ValueError(
                    "invalid pickup_lift_settle_s"
                )

            print(
                f"[STACK_LIFT][{label}] "
                f"position={position:+.2f} "
                f"settle={settle_s:.2f}s"
            )

            if settle_s > 0.0:
                io.sleep(
                    settle_s
                )

            return True

        except (
            AttributeError,
            KeyError,
            TypeError,
            ValueError,
            RuntimeError,
        ) as exc:
            print(
                f"[STACK_LIFT][{label}] "
                f"FAILED: {exc}"
            )

            self.status = (
                BehaviorStatus.FAILED
            )

            return False

    def _start_commitment(
        self,
        *,
        motion_backend,
    ):
        print(
            "[STACK_OBJECT] "
            "LOW-cube final commitment "
            f"d={self.final_drive_mm:.1f}mm"
        )

        self._drive = Drive(
            distance_mm=self.final_drive_mm
        )

        self._drive.start(
            motion_backend=motion_backend
        )

        self._step = "DRIVE"

        return self.status

    def _start_release(
        self,
        *,
        lvl2,
    ):
        print(
            "[STACK_OBJECT] release carried cube"
        )

        self._release = Release()

        self._release.start(
            lvl2=lvl2
        )

        self._step = "RELEASE"

        return self.status

    def _start_retreat(
        self,
        *,
        lvl2,
        motion_backend,
    ):
        # Mirror HIGH pickup clearance for the initial implementation.
        #
        # The stack placement height itself remains independently
        # configured through lift_stack_position.
        if not self._command_lift(
            lvl2,
            "lift_high_retreat_position",
            "RETREAT",
        ):
            return self.status

        print(
            "[STACK_OBJECT] reverse full commitment "
            f"d={-self.final_drive_mm:.1f}mm"
        )

        self._retreat = Drive(
            distance_mm=-self.final_drive_mm
        )

        self._retreat.start(
            motion_backend=motion_backend
        )

        self._step = "RETREAT"

        return self.status

    def update(
        self,
        *,
        lvl2,
        motion_backend,
        **_,
    ):
        if (
            self.status
            != BehaviorStatus.RUNNING
        ):
            return self.status

        # --------------------------------------------------
        # Final alignment
        # --------------------------------------------------

        if self._step == "ALIGN":
            st = self._align.update(
                motion_backend=motion_backend
            )

            if st == PrimitiveStatus.RUNNING:
                return self.status

            if st == PrimitiveStatus.FAILED:
                print(
                    "[STACK_OBJECT] "
                    "final alignment FAILED"
                )

                self.status = (
                    BehaviorStatus.FAILED
                )

                return self.status

            print(
                "[STACK_OBJECT] "
                "final alignment complete"
            )

            return self._start_commitment(
                motion_backend=motion_backend
            )

        # --------------------------------------------------
        # LOW-cube final commitment
        # --------------------------------------------------

        if self._step == "DRIVE":
            st = self._drive.update(
                motion_backend=motion_backend
            )

            if st == PrimitiveStatus.RUNNING:
                return self.status

            if st == PrimitiveStatus.FAILED:
                print(
                    "[STACK_OBJECT] "
                    "final commitment FAILED"
                )

                self.status = (
                    BehaviorStatus.FAILED
                )

                return self.status

            print(
                "[STACK_OBJECT] "
                "final commitment complete"
            )

            return self._start_release(
                lvl2=lvl2
            )

        # --------------------------------------------------
        # Release
        # --------------------------------------------------

        if self._step == "RELEASE":
            st = self._release.update()

            if st == PrimitiveStatus.RUNNING:
                return self.status

            if st == PrimitiveStatus.FAILED:
                print(
                    "[STACK_OBJECT] "
                    "release FAILED"
                )

                self.status = (
                    BehaviorStatus.FAILED
                )

                return self.status

            self.object_released = True

            print(
                "[STACK_OBJECT] "
                "release complete"
            )

            return self._start_retreat(
                lvl2=lvl2,
                motion_backend=motion_backend,
            )

        # --------------------------------------------------
        # Retreat
        # --------------------------------------------------

        if self._step == "RETREAT":
            st = self._retreat.update(
                motion_backend=motion_backend
            )

            if st == PrimitiveStatus.RUNNING:
                return self.status

            if st == PrimitiveStatus.FAILED:
                print(
                    "[STACK_OBJECT] "
                    "retreat FAILED"
                )

                self.status = (
                    BehaviorStatus.FAILED
                )

                return self.status

            print(
                "[STACK_OBJECT] complete"
            )

            self.status = (
                BehaviorStatus.SUCCEEDED
            )

            return self.status

        self.status = (
            BehaviorStatus.FAILED
        )

        return self.status

    def stop(
        self,
        *,
        motion_backend=None,
        **_,
    ):
        for child in (
            self._align,
            self._drive,
            self._release,
            self._retreat,
        ):
            if child is None:
                continue

            try:
                if motion_backend is not None:
                    child.stop(
                        motion_backend=motion_backend
                    )
                else:
                    child.stop()

            except TypeError:
                try:
                    child.stop()
                except Exception:
                    pass

            except Exception:
                pass