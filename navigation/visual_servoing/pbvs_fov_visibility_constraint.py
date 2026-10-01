# navigation/visual_servoing/pbvs_fov_visibility_constraint.py

from __future__ import annotations

from dataclasses import dataclass
import math

from navigation.visual_servoing.servoing_types import NonHolonomicVelocity2D


@dataclass(frozen=True)
class PBVSFovVisibilityConstraintParams:
    half_fov_rad: float
    activation_margin_rad: float
    linear_scale_mps: float
    angular_scale_rps: float


@dataclass(frozen=True)
class PBVSFovVisibilityConstraintResult:
    active: bool
    outward: bool
    feature_bearing_rad: float
    feature_range_m: float
    safe_half_fov_rad: float
    weight: float
    nominal_bearing_rate_rps: float
    constrained_bearing_rate_rps: float
    velocity: NonHolonomicVelocity2D


class PBVSFovVisibilityConstraint:
    """
    Math-only horizontal FOV constraint for nonholonomic PBVS.

    Kermorgant/Chaumette basis:
    - zero constraint weight in the safe image region
    - increasing weight as a feature approaches the image boundary
    - no intervention when the nominal task already moves the feature
      away from the boundary

    Robot/image conventions:
        bearing > 0 : feature to camera right
        omega > 0   : robot turns left / CCW
        v > 0       : robot drives forward

    For a stationary feature:
        bearing_dot = (sin(bearing) / range) * v + omega

    Supply the most endangered horizontal image feature. For an
    AprilTag this should ideally be a tag corner, not only its centre.
    """

    _EPSILON = 1e-12

    def __init__(self, params: PBVSFovVisibilityConstraintParams):
        self.params = params
        self._validate()

    @property
    def safe_half_fov_rad(self) -> float:
        return (
            self.params.half_fov_rad
            - self.params.activation_margin_rad
        )

    def calculate(
        self,
        *,
        feature_bearing_rad: float,
        feature_range_m: float,
        nominal_velocity: NonHolonomicVelocity2D,
    ) -> PBVSFovVisibilityConstraintResult:

        b = float(feature_bearing_rad)
        r = float(feature_range_m)
        v0 = float(nominal_velocity.linear_mps)
        w0 = float(nominal_velocity.angular_rps)

        if not all(math.isfinite(x) for x in (b, r, v0, w0)):
            raise ValueError("FOV constraint inputs must be finite")

        if r <= 0.0:
            raise ValueError("feature_range_m must be > 0")

        # Differential-drive horizontal image-feature interaction.
        interaction_v = math.sin(b) / r

        nominal_rate = (
            interaction_v * v0
            + w0
        )

        # Is PBVS currently driving the feature toward its FOV edge?
        outward = (
            (b > 0.0 and nominal_rate > 0.0)
            or
            (b < 0.0 and nominal_rate < 0.0)
        )

        abs_b = abs(b)
        safe = self.safe_half_fov_rad
        hard = self.params.half_fov_rad

        # PBVS remains completely untouched in the safe region,
        # or whenever it is already moving the feature inward.
        if not outward or abs_b <= safe:
            return self._result(
                active=False,
                outward=outward,
                b=b,
                r=r,
                weight=0.0,
                nominal_rate=nominal_rate,
                constrained_rate=nominal_rate,
                v=v0,
                w=w0,
            )

        if abs_b >= hard - self._EPSILON:

            # Infinite-weight limit:
            # minimum change from PBVS giving bearing_dot = 0.
            v, w = self._zero_rate_projection(
                interaction_v,
                v0,
                w0,
            )

            weight = math.inf

        else:

            # Kermorgant/Chaumette-style generic visibility weight:
            #
            #   0          at safe boundary
            #   increasing through activation region
            #   -> infinity at physical FOV edge
            weight = (
                (abs_b - safe)
                /
                (hard - abs_b)
            )

            v, w = self._weighted_correction(
                interaction_v,
                v0,
                w0,
                weight,
            )

        # --------------------------------------------------
        # Preserve PBVS travel direction
        # --------------------------------------------------
        #
        # The unconstrained visibility optimisation is allowed
        # mathematically to reverse linear velocity if that is
        # the cheapest way to reduce outward image motion.
        #
        # That is undesirable here: PBVS has already selected a
        # persistent forward/reverse direction for the manoeuvre.
        # FOV protection may slow that motion to zero, but must
        # not reverse it.
        #
        # If the unconstrained optimum crosses v=0, the optimum
        # subject to the travel-direction constraint lies on the
        # boundary v=0. Re-solve the angular part there rather
        # than merely clamping v while retaining an incompatible
        # omega.
        if (
            abs(v0) > self._EPSILON
            and v * v0 < 0.0
        ):
            v = 0.0

            if math.isinf(weight):
                # At the physical FOV boundary, bearing_dot must
                # be zero. With v=0 this also requires omega=0.
                w = 0.0
            else:
                # With v constrained to zero, minimise:
                #
                #   (w/W - w0/W)^2
                #   + h^2 * (w/W)^2
                #
                # giving:
                #
                #   w = w0 / (1 + h^2)
                h2 = weight * weight
                w = w0 / (1.0 + h2)

        constrained_rate = (
            interaction_v * v
            + w
        )

        return self._result(
            active=True,
            outward=True,
            b=b,
            r=r,
            weight=weight,
            nominal_rate=nominal_rate,
            constrained_rate=constrained_rate,
            v=v,
            w=w,
        )

    def _weighted_correction(
        self,
        interaction_v: float,
        v0: float,
        w0: float,
        weight: float,
    ) -> tuple[float, float]:
        """
        Minimum-intervention weighted least squares.

        Normalised velocity:

            z = [v / V, omega / W]

        Minimise:

            ||z - z_pbvs||^2
            +
            h^2 * (bearing_dot / W)^2

        The first term preserves PBVS.
        The second progressively suppresses outward image motion.
        """

        v_scale = self.params.linear_scale_mps
        w_scale = self.params.angular_scale_rps

        z_v = v0 / v_scale
        z_w = w0 / w_scale

        j_v = (
            interaction_v
            * v_scale
            / w_scale
        )

        j_dot_z = (
            j_v * z_v
            + z_w
        )

        j_norm_sq = (
            j_v * j_v
            + 1.0
        )

        h2 = weight * weight

        correction = (
            h2
            * j_dot_z
            /
            (1.0 + h2 * j_norm_sq)
        )

        z_v -= correction * j_v
        z_w -= correction

        return (
            z_v * v_scale,
            z_w * w_scale,
        )

    def _zero_rate_projection(
        self,
        interaction_v: float,
        v0: float,
        w0: float,
    ) -> tuple[float, float]:
        """
        Infinite-weight limit.

        Find the minimum normalised change from PBVS subject to:

            bearing_dot = 0
        """

        v_scale = self.params.linear_scale_mps
        w_scale = self.params.angular_scale_rps

        z_v = v0 / v_scale
        z_w = w0 / w_scale

        j_v = (
            interaction_v
            * v_scale
            / w_scale
        )

        j_dot_z = (
            j_v * z_v
            + z_w
        )

        j_norm_sq = (
            j_v * j_v
            + 1.0
        )

        correction = (
            j_dot_z
            / j_norm_sq
        )

        z_v -= correction * j_v
        z_w -= correction

        return (
            z_v * v_scale,
            z_w * w_scale,
        )

    def _result(
        self,
        *,
        active: bool,
        outward: bool,
        b: float,
        r: float,
        weight: float,
        nominal_rate: float,
        constrained_rate: float,
        v: float,
        w: float,
    ) -> PBVSFovVisibilityConstraintResult:

        return PBVSFovVisibilityConstraintResult(
            active=active,
            outward=outward,
            feature_bearing_rad=b,
            feature_range_m=r,
            safe_half_fov_rad=self.safe_half_fov_rad,
            weight=weight,
            nominal_bearing_rate_rps=nominal_rate,
            constrained_bearing_rate_rps=constrained_rate,
            velocity=NonHolonomicVelocity2D(
                linear_mps=v,
                angular_rps=w,
            ),
        )

    def _validate(self) -> None:

        p = self.params

        if not all(
            math.isfinite(x)
            for x in (
                p.half_fov_rad,
                p.activation_margin_rad,
                p.linear_scale_mps,
                p.angular_scale_rps,
            )
        ):
            raise ValueError(
                "all FOV constraint parameters must be finite"
            )

        if not (
            0.0
            < p.half_fov_rad
            < math.pi / 2.0
        ):
            raise ValueError(
                "half_fov_rad must be in (0, pi/2)"
            )

        if not (
            0.0
            < p.activation_margin_rad
            < p.half_fov_rad
        ):
            raise ValueError(
                "activation_margin_rad must be > 0 "
                "and < half_fov_rad"
            )

        if p.linear_scale_mps <= 0.0:
            raise ValueError(
                "linear_scale_mps must be > 0"
            )

        if p.angular_scale_rps <= 0.0:
            raise ValueError(
                "angular_scale_rps must be > 0"
            )