"""Simulated Qube-Servo 3 implementing the common QubeInterface."""

import numpy as np
from interface import QubeInterface, QubeState
from sim.plant import QubeParams, rk4_step


class SimulatedQube(QubeInterface):
    """Physics simulation of the Qube-Servo 3 + inverted pendulum.

    Uses RK4 integration of the nonlinear EOM. Supports sub-stepping
    for better accuracy at larger control dt.

    Args:
        dt:            Control loop period [s]. Default 2 ms (500 Hz).
        params:        Plant parameters (use defaults or supply identified values).
        initial_alpha: Starting pendulum angle from upright [rad].
                       pi = hanging down, 0.1 = near-upright.
        substeps:      Number of RK4 sub-steps per control step.
        sensor_noise:  Stddev of additive Gaussian noise on encoder readings [rad].
    """

    def __init__(
        self,
        dt: float = 0.002,
        params: QubeParams | None = None,
        initial_alpha: float = np.pi,
        substeps: int = 10,
        sensor_noise: float = 0.0,
    ):
        self._dt = dt
        self._params = params or QubeParams()
        self._initial_alpha = initial_alpha
        self._substeps = substeps
        self._sensor_noise = sensor_noise
        self._state = np.zeros(4)
        self.reset()

    @property
    def dt(self) -> float:
        return self._dt

    def reset(self) -> QubeState:
        self._state = np.array([0.0, self._initial_alpha, 0.0, 0.0])
        return self._read_state()

    def step(self, voltage: float) -> QubeState:
        voltage = float(np.clip(voltage, -10.0, 10.0))
        sub_dt = self._dt / self._substeps
        for _ in range(self._substeps):
            self._state = rk4_step(self._state, voltage, sub_dt, self._params)

        # Wrap angles to [-pi, pi]
        self._state[0] = _wrap_angle(self._state[0])
        self._state[1] = _wrap_angle(self._state[1])

        return self._read_state()

    def close(self) -> None:
        pass  # Nothing to release in simulation

    def _read_state(self) -> QubeState:
        s = self._state.copy()
        if self._sensor_noise > 0:
            s[0] += np.random.normal(0, self._sensor_noise)
            s[1] += np.random.normal(0, self._sensor_noise)
        return QubeState.from_array(s)


def _wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi
