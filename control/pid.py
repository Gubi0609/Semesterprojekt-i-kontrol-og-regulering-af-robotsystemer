"""PID controller with anti-windup, derivative filtering, and output clamping."""

import numpy as np


class PID:
    """Discrete PID controller.

    Args:
        kp: Proportional gain.
        ki: Integral gain.
        kd: Derivative gain.
        dt: Timestep [s].
        output_limit: Symmetric saturation limit on output.
        integral_limit: Anti-windup clamp on integrator state.
        d_filter_coeff: First-order filter on derivative term (0 = no filter, 0.9 = heavy).
            Implements: d_filtered = coeff * d_prev + (1 - coeff) * d_raw
    """

    def __init__(
        self,
        kp: float = 0.0,
        ki: float = 0.0,
        kd: float = 0.0,
        dt: float = 0.002,
        output_limit: float = 10.0,
        integral_limit: float = 5.0,
        d_filter_coeff: float = 0.8,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.d_filter_coeff = d_filter_coeff

        self._integral = 0.0
        self._prev_error = None
        self._d_filtered = 0.0

    def compute(self, error: float) -> float:
        """Compute one PID step.

        Args:
            error: setpoint - measurement (positive error → positive output).

        Returns:
            Control output (clamped to +/- output_limit).
        """
        # Proportional
        p_term = self.kp * error

        # Integral with anti-windup
        self._integral += error * self.dt
        self._integral = np.clip(self._integral, -self.integral_limit, self.integral_limit)
        i_term = self.ki * self._integral

        # Derivative (on error, with low-pass filter)
        if self._prev_error is None:
            d_raw = 0.0
        else:
            d_raw = (error - self._prev_error) / self.dt

        self._d_filtered = (
            self.d_filter_coeff * self._d_filtered
            + (1.0 - self.d_filter_coeff) * d_raw
        )
        d_term = self.kd * self._d_filtered
        self._prev_error = error

        output = p_term + i_term + d_term
        return float(np.clip(output, -self.output_limit, self.output_limit))

    def reset(self):
        """Reset integrator and derivative state."""
        self._integral = 0.0
        self._prev_error = None
        self._d_filtered = 0.0
