# skills/algorithm/PID.py

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal


DerivativeMode = Literal["error", "measurement"]


# ==================================================
# PID state
# ==================================================

@dataclass
class PIDState:
    """
    Internal history for one independent PID control loop.

    A single PIDController can service many control loops.

    Each loop_id receives its own PIDState so integral and
    derivative history are never shared between unrelated loops.
    """

    integral: float = 0.0

    previous_error: float | None = None
    previous_measurement: float | None = None
    previous_timestamp: float | None = None


# ==================================================
# PID result
# ==================================================

@dataclass(frozen=True)
class PIDResult:
    """
    Result of one PID calculation.

    Individual terms are returned to support diagnostics,
    logging, tuning, and analysis.
    """

    output: float

    setpoint: float
    measurement: float
    error: float

    p_term: float
    i_term: float
    d_term: float

    dt: float


# ==================================================
# PID controller
# ==================================================

class PIDController:
    """
    Generic shared PID algorithm.

    PIDController contains no application policy.

    It does not choose:

        - setpoint
        - measurement
        - units
        - Kp
        - Ki
        - Kd
        - derivative mode
        - integral limits
        - output limits

    Those decisions belong entirely to the calling process
    and its configuration.

    One PIDController instance may service many independent
    control loops.

    Each loop is identified by loop_id and maintains its own:

        - integral accumulator
        - previous error
        - previous measurement
        - previous timestamp
    """

    def __init__(self):
        self._states: dict[str, PIDState] = {}

    # ==================================================
    # Update
    # ==================================================

    def update(
        self,
        *,
        loop_id: str,
        setpoint: float,
        measurement: float,
        timestamp: float,
        kp: float,
        ki: float,
        kd: float,
        derivative_mode: DerivativeMode,
        integral_min: float | None = None,
        integral_max: float | None = None,
        output_min: float | None = None,
        output_max: float | None = None,
    ) -> PIDResult:
        """
        Perform one PID update.

        Parameters
        ----------
        loop_id:
            Unique identifier for this control loop.

            Examples:

                "servoing.approach_target.linear"
                "servoing.approach_target.angular"
                "servoing.return_to_base.linear"
                "servoing.return_to_base.angular"
                "lift.position"

            Each loop_id maintains completely independent state.

        setpoint:
            Desired value.

        measurement:
            Current measured value.

        timestamp:
            Timestamp in seconds.

            A particular loop_id must always use the same
            timestamp timebase.

        kp:
            Proportional gain.

        ki:
            Integral gain.

        kd:
            Derivative gain.

        derivative_mode:
            Determines how the derivative is calculated.

            "error"

                derivative =
                    change in error / change in time

            "measurement"

                derivative =
                    -change in measurement / change in time

            The negative sign in measurement mode preserves:

                error = setpoint - measurement

        integral_min:
            Optional minimum value for accumulated integral state.

            None means no minimum limit.

        integral_max:
            Optional maximum value for accumulated integral state.

            None means no maximum limit.

        output_min:
            Optional minimum PID output.

            None means no minimum limit.

        output_max:
            Optional maximum PID output.

            None means no maximum limit.

        Returns
        -------
        PIDResult
            PID output together with the individual P, I and D
            terms and diagnostic values.
        """

        # --------------------------------------------------
        # Validate inputs
        # --------------------------------------------------

        if not loop_id:
            raise ValueError(
                "loop_id must not be empty"
            )

        if derivative_mode not in (
            "error",
            "measurement",
        ):
            raise ValueError(
                "derivative_mode must be "
                "'error' or 'measurement'"
            )

        self._validate_limits(
            minimum=integral_min,
            maximum=integral_max,
            name="integral",
        )

        self._validate_limits(
            minimum=output_min,
            maximum=output_max,
            name="output",
        )

        # --------------------------------------------------
        # Normalise numeric inputs
        # --------------------------------------------------

        setpoint = float(setpoint)
        measurement = float(measurement)
        timestamp = float(timestamp)

        kp = float(kp)
        ki = float(ki)
        kd = float(kd)

        # --------------------------------------------------
        # Current error
        # --------------------------------------------------

        error = (
            setpoint
            - measurement
        )

        # --------------------------------------------------
        # Retrieve independent state for this loop
        # --------------------------------------------------

        state = self._states.setdefault(
            loop_id,
            PIDState(),
        )

        dt = 0.0
        derivative = 0.0

        # --------------------------------------------------
        # Time-dependent terms
        # --------------------------------------------------

        if state.previous_timestamp is not None:

            candidate_dt = (
                timestamp
                - state.previous_timestamp
            )

            # Only advance I and D using a genuinely newer
            # sample.
            if candidate_dt > 0.0:

                dt = candidate_dt

                # ------------------------------------------
                # Integral
                # ------------------------------------------

                state.integral += (
                    error
                    * dt
                )

                state.integral = self._clamp(
                    state.integral,
                    minimum=integral_min,
                    maximum=integral_max,
                )

                # ------------------------------------------
                # Derivative
                # ------------------------------------------

                if derivative_mode == "error":

                    if state.previous_error is not None:

                        derivative = (
                            error
                            - state.previous_error
                        ) / dt

                elif derivative_mode == "measurement":

                    if (
                        state.previous_measurement
                        is not None
                    ):

                        derivative = -(
                            measurement
                            - state.previous_measurement
                        ) / dt

        # --------------------------------------------------
        # PID terms
        # --------------------------------------------------

        p_term = (
            kp
            * error
        )

        i_term = (
            ki
            * state.integral
        )

        d_term = (
            kd
            * derivative
        )

        # --------------------------------------------------
        # PID output
        # --------------------------------------------------

        output = (
            p_term
            + i_term
            + d_term
        )

        output = self._clamp(
            output,
            minimum=output_min,
            maximum=output_max,
        )

        # --------------------------------------------------
        # Save loop history
        #
        # Repeated or older timestamps are not allowed to
        # replace valid history.
        # --------------------------------------------------

        if (
            state.previous_timestamp is None
            or timestamp > state.previous_timestamp
        ):
            state.previous_error = error

            state.previous_measurement = (
                measurement
            )

            state.previous_timestamp = (
                timestamp
            )

        # --------------------------------------------------
        # Result
        # --------------------------------------------------

        return PIDResult(
            output=output,

            setpoint=setpoint,
            measurement=measurement,
            error=error,

            p_term=p_term,
            i_term=i_term,
            d_term=d_term,

            dt=dt,
        )

    # ==================================================
    # State management
    # ==================================================

    def reset(
        self,
        loop_id: str,
    ) -> None:
        """
        Reset one PID loop's history.

        The next update for this loop behaves as its first sample.
        """

        self._states.pop(
            loop_id,
            None,
        )

    def reset_all(
        self,
    ) -> None:
        """
        Reset PID history for every control loop.
        """

        self._states.clear()

    def has_state(
        self,
        loop_id: str,
    ) -> bool:
        """
        Return True if the named loop currently has PID history.
        """

        return (
            loop_id
            in self._states
        )

    # ==================================================
    # Helpers
    # ==================================================

    @staticmethod
    def _clamp(
        value: float,
        *,
        minimum: float | None,
        maximum: float | None,
    ) -> float:
        """
        Apply optional lower and upper limits.
        """

        if minimum is not None:
            value = max(
                float(minimum),
                value,
            )

        if maximum is not None:
            value = min(
                float(maximum),
                value,
            )

        return value

    @staticmethod
    def _validate_limits(
        *,
        minimum: float | None,
        maximum: float | None,
        name: str,
    ) -> None:
        """
        Reject reversed limit pairs.
        """

        if (
            minimum is not None
            and maximum is not None
            and float(minimum)
            > float(maximum)
        ):
            raise ValueError(
                f"{name}_min must be "
                f"<= {name}_max"
            )


# ==================================================
# Shared PID engine
# ==================================================
#
# One PID engine can be used throughout the robot process.
#
# Independent applications and control axes remain isolated
# because PIDController stores separate state for every loop_id.
# ==================================================

SHARED_PID = PIDController()