"""Abstract interface that both simulation and hardware implement."""

from abc import ABC, abstractmethod
from dataclasses import dataclass
import numpy as np


@dataclass
class QubeState:
    """Full state of the Qube-Servo 3 with pendulum.

    Conventions:
        theta:     Motor/arm angle [rad]
        alpha:     Pendulum angle from upright [rad] (0 = balanced upright)
        theta_dot: Motor angular velocity [rad/s]
        alpha_dot: Pendulum angular velocity [rad/s]
    """
    theta: float = 0.0
    alpha: float = 0.0
    theta_dot: float = 0.0
    alpha_dot: float = 0.0

    def as_array(self) -> np.ndarray:
        return np.array([self.theta, self.alpha, self.theta_dot, self.alpha_dot])

    @classmethod
    def from_array(cls, x: np.ndarray) -> "QubeState":
        return cls(theta=x[0], alpha=x[1], theta_dot=x[2], alpha_dot=x[3])


class QubeInterface(ABC):
    """Common interface for simulated and real Qube-Servo 3."""

    @abstractmethod
    def reset(self) -> QubeState:
        """Reset/initialize the system. Returns initial state."""
        ...

    @abstractmethod
    def step(self, voltage: float) -> QubeState:
        """Apply motor voltage and advance one timestep.

        Args:
            voltage: Motor command in volts. Clamped to [-10, 10].

        Returns:
            Updated state after one dt.
        """
        ...

    @abstractmethod
    def close(self) -> None:
        """Release resources (safe motor shutdown on hardware)."""
        ...

    @property
    @abstractmethod
    def dt(self) -> float:
        """Control loop period in seconds."""
        ...
