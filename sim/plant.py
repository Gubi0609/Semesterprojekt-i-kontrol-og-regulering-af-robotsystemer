"""Nonlinear dynamics of the Qube-Servo 3 rotary inverted pendulum.

Equations of motion derived via Euler-Lagrange for two-link planar
manipulator with one actuated joint (motor) and one passive joint (pendulum).

State vector: x = [theta, alpha, theta_dot, alpha_dot]
    theta   - motor (arm) angle [rad]
    alpha   - pendulum angle from upright [rad]  (alpha=0 is balanced)
    theta_dot - motor angular velocity [rad/s]
    alpha_dot - pendulum angular velocity [rad/s]

Input: Vm - motor voltage [V]

References:
    Quanser QUBE-Servo 2/3 courseware & workbook (Rotary Pendulum)
    Astrom & Murray, "Feedback Systems" (Ch. 3 - rotary inverted pendulum)
"""

import numpy as np
from dataclasses import dataclass, field


@dataclass
class QubeParams:
    """Physical parameters of the Qube-Servo 3 + pendulum module.

    Default values are from the Quanser Qube-Servo 3 datasheet and
    rotary pendulum courseware. Adjust if you do system identification
    on your specific unit.
    """
    # Motor electrical
    Rm: float = 8.4        # Terminal resistance [Ohm]
    km: float = 0.042      # Motor back-EMF / torque constant [V/(rad/s)] = [N·m/A]

    # Rotor (motor hub + arm disc)
    Jr: float = 4.0e-6     # Rotor moment of inertia [kg·m^2]
    Lr: float = 0.0826     # Arm length (pivot to pendulum axis) [m]
    Dr: float = 0.0       # Arm viscous damping [N·m·s/rad]  (small, often ~0)

    # Pendulum
    mp: float = 0.024      # Pendulum mass [kg]
    Lp: float = 0.129      # Pendulum full length [m]
    Jp: float = 3.33e-5    # Pendulum moment of inertia about pivot [kg·m^2]
    Dp: float = 0.0       # Pendulum viscous damping [N·m·s/rad]

    g: float = 9.81        # Gravitational acceleration [m/s^2]

    # Derived (computed post-init)
    half_Lp: float = field(init=False)
    Jt: float = field(init=False)

    def __post_init__(self):
        self.half_Lp = self.Lp / 2.0
        # Total effective inertia (used in EOM denominator)
        self.Jt = self.Jr + self.mp * self.Lr**2


def dynamics(x: np.ndarray, Vm: float, p: QubeParams) -> np.ndarray:
    """Compute dx/dt for the rotary inverted pendulum.

    Uses the standard Euler-Lagrange formulation:

        M(q) * q_ddot + C(q, q_dot) * q_dot + G(q) = tau

    where q = [theta, alpha].

    Args:
        x:  State [theta, alpha, theta_dot, alpha_dot].
        Vm: Motor voltage [V] (will be clamped to +/-10 V internally).
        p:  System parameters.

    Returns:
        dx/dt as [theta_dot, alpha_dot, theta_ddot, alpha_ddot].
    """
    Vm = np.clip(Vm, -10.0, 10.0)

    theta, alpha, theta_dot, alpha_dot = x

    sin_a = np.sin(alpha)
    cos_a = np.cos(alpha)

    # Motor torque from applied voltage (accounting for back-EMF)
    tau = (p.km * (Vm - p.km * theta_dot)) / p.Rm

    # Mass matrix elements  M(q) * [theta_ddot, alpha_ddot]^T = rhs
    #   M = [[M11, M12],
    #        [M21, M22]]
    M11 = p.Jt + p.Jp * sin_a**2
    M12 = -p.mp * p.half_Lp * p.Lr * cos_a
    M21 = M12
    M22 = p.Jp + p.mp * p.half_Lp**2

    # Right-hand side (torques minus Coriolis/centrifugal minus gravity minus damping)
    rhs1 = (tau
            - p.Dr * theta_dot
            - p.mp * p.half_Lp * p.Lr * alpha_dot**2 * sin_a
            - 2.0 * p.Jp * sin_a * cos_a * theta_dot * alpha_dot)

    rhs2 = (-p.Dp * alpha_dot
            + p.mp * p.half_Lp * p.g * sin_a
            + p.Jp * sin_a * cos_a * theta_dot**2)

    # Solve 2x2 system: [theta_ddot, alpha_ddot] = M^{-1} * rhs
    det = M11 * M22 - M12 * M21
    theta_ddot = (M22 * rhs1 - M12 * rhs2) / det
    alpha_ddot = (M11 * rhs2 - M21 * rhs1) / det

    return np.array([theta_dot, alpha_dot, theta_ddot, alpha_ddot])


def rk4_step(x: np.ndarray, Vm: float, dt: float, p: QubeParams) -> np.ndarray:
    """Single RK4 integration step (much more accurate than Euler for stiff pendulum dynamics)."""
    k1 = dynamics(x, Vm, p)
    k2 = dynamics(x + 0.5 * dt * k1, Vm, p)
    k3 = dynamics(x + 0.5 * dt * k2, Vm, p)
    k4 = dynamics(x + dt * k3, Vm, p)
    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
