"""Energy-based swing-up controller for the inverted pendulum.

Uses Astrom's energy-pumping method with arm angle limiting to prevent
the motor from hitting mechanical endstops.

The key safety addition: when the arm approaches the angle limit, the
controller either reduces output or actively brakes, preventing damage
to the Qube-Servo 3 hardware.
"""

import numpy as np
from sim.plant import QubeParams


class SwingUpController:
    """Energy-based swing-up with arm protection.

    The pendulum energy relative to upright is:
        E = 0.5 * Jp * alpha_dot^2 - mp * g * (Lp/2) * (cos(alpha) - 1)

    At the upright equilibrium, E = 0. When hanging down (alpha=pi), E < 0.
    We pump energy by applying: u = sat(mu * sign(E_err * alpha_dot * cos(alpha)))

    Arm protection:
        - Soft limit: reduce swing-up voltage linearly as arm approaches limit
        - Hard limit: override with a restoring voltage to push arm back
        - This keeps the arm within a safe angular range at all times

    Args:
        params:         Plant physical parameters.
        mu:             Swing-up gain (voltage amplitude). Lower = gentler but slower.
        catch_angle:    When |alpha| < catch_angle, switch to balance [rad].
        arm_limit:      Max allowed arm angle [rad]. Default ~90° — safe margin
                        from any endstop.
        arm_soft_zone:  Degrees before the limit where voltage starts tapering [rad].
        arm_brake_gain: Proportional gain for the arm-limiting restoring torque.
    """

    def __init__(
        self,
        params: QubeParams | None = None,
        mu: float = 3.0,
        catch_angle: float = np.radians(20),
        arm_limit: float = np.radians(90),
        arm_soft_zone: float = np.radians(30),
        arm_brake_gain: float = 5.0,
    ):
        self.p = params or QubeParams()
        self.mu = mu
        self.catch_angle = catch_angle
        self.arm_limit = arm_limit
        self.arm_soft_zone = arm_soft_zone
        self.arm_brake_gain = arm_brake_gain

    def compute(self, theta: float, alpha: float,
                theta_dot: float, alpha_dot: float) -> float:
        """Compute swing-up voltage with arm protection.

        Args:
            theta:     Arm angle [rad].
            alpha:     Pendulum angle from upright [rad].
            theta_dot: Arm angular velocity [rad/s].
            alpha_dot: Pendulum angular velocity [rad/s].

        Returns:
            Motor voltage [V], clamped to +/- 10V.
        """
        # --- Energy-based swing-up ---
        E = (0.5 * self.p.Jp * alpha_dot**2
             - self.p.mp * self.p.g * self.p.half_Lp * (np.cos(alpha) - 1.0))
        E_err = E - 0.0  # target energy = 0 (upright equilibrium)

        u_swing = self.mu * np.sign(E_err * alpha_dot * np.cos(alpha))

        # --- Arm angle protection ---
        abs_theta = abs(theta)

        if abs_theta > self.arm_limit:
            # Hard limit: override with restoring voltage + velocity damping
            u = -self.arm_brake_gain * theta - 2.0 * theta_dot
        elif abs_theta > (self.arm_limit - self.arm_soft_zone):
            # Soft zone: taper swing-up voltage and blend in restoring force
            # scale goes from 1.0 (at soft zone entry) to 0.0 (at hard limit)
            scale = (self.arm_limit - abs_theta) / self.arm_soft_zone
            scale = np.clip(scale, 0.0, 1.0)

            # Also check: if swing-up would push arm further out, suppress it
            pushing_outward = (np.sign(u_swing) == np.sign(theta))
            if pushing_outward:
                scale *= 0.3  # heavily reduce outward commands near limit

            u_restore = -1.0 * theta  # gentle centering force
            u = scale * u_swing + (1.0 - scale) * u_restore
        else:
            # Safe zone: full swing-up authority
            u = u_swing

        return float(np.clip(u, -10.0, 10.0))

    def should_catch(self, alpha: float) -> bool:
        """Returns True when pendulum is close enough to upright for balance controller."""
        return abs(alpha) < self.catch_angle
