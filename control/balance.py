"""Balance controller for the inverted pendulum.

Implements full-state feedback (derived from LQR) with optional
integral action on alpha for steady-state error rejection.

This is more appropriate than a naive PID for the rotary inverted
pendulum because:
  1. The system has 4 states, not 1 — a single PID on alpha ignores
     arm angle and both velocities.
  2. Velocity is available from encoders / sim, so we don't need to
     re-differentiate (which adds lag).

The controller computes:
    u = -k_theta * theta - k_alpha * alpha
        -k_theta_dot * theta_dot - k_alpha_dot * alpha_dot
        -k_integral * integral(alpha)

Default gains are from LQR with Q=diag(10,100,1,1), R=1.
"""

import numpy as np
from interface import QubeState


class BalanceController:
    """Full-state-feedback balance controller with integral action.

    Args:
        k_theta:     Gain on arm angle [V/rad].
        k_alpha:     Gain on pendulum angle [V/rad].
        k_theta_dot: Gain on arm velocity [V/(rad/s)].
        k_alpha_dot: Gain on pendulum velocity [V/(rad/s)].
        k_integral:  Integral gain on alpha for offset rejection [V/(rad·s)].
        dt:          Timestep [s].
        voltage_limit: Output saturation [V].
        integral_limit: Anti-windup clamp [rad·s].
    """

    def __init__(
        self,
        k_theta: float = -3.16,
        k_alpha: float = 49.0,
        k_theta_dot: float = -1.83,
        k_alpha_dot: float = 4.32,
        k_integral: float = 2.0,
        dt: float = 0.002,
        voltage_limit: float = 10.0,
        integral_limit: float = 0.5,
    ):
        self.k_theta = k_theta
        self.k_alpha = k_alpha
        self.k_theta_dot = k_theta_dot
        self.k_alpha_dot = k_alpha_dot
        self.k_integral = k_integral
        self.dt = dt
        self.voltage_limit = voltage_limit
        self.integral_limit = integral_limit

        self._alpha_integral = 0.0

    def compute(self, state: QubeState) -> float:
        """Compute motor voltage from full state.

        Args:
            state: Current state (theta, alpha, theta_dot, alpha_dot).

        Returns:
            Motor voltage [V], clamped to +/- voltage_limit.
        """
        # Integrate alpha for steady-state rejection
        self._alpha_integral += state.alpha * self.dt
        self._alpha_integral = np.clip(
            self._alpha_integral, -self.integral_limit, self.integral_limit
        )

        u = (
            -self.k_theta * state.theta
            - self.k_alpha * state.alpha
            - self.k_theta_dot * state.theta_dot
            - self.k_alpha_dot * state.alpha_dot
            - self.k_integral * self._alpha_integral
        )

        return float(np.clip(u, -self.voltage_limit, self.voltage_limit))

    def reset(self):
        """Reset integral state."""
        self._alpha_integral = 0.0

    @property
    def gains_array(self) -> np.ndarray:
        """Return gains as array [k_theta, k_alpha, k_theta_dot, k_alpha_dot]."""
        return np.array([self.k_theta, self.k_alpha, self.k_theta_dot, self.k_alpha_dot])
