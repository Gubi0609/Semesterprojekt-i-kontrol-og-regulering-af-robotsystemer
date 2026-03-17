"""Signal processing utilities for encoder signals."""

import numpy as np


class LowPassFilter:
    """First-order IIR low-pass filter.

    y[n] = alpha * y[n-1] + (1 - alpha) * x[n]

    Args:
        cutoff_hz: Cutoff frequency [Hz].
        dt: Sample period [s].
    """

    def __init__(self, cutoff_hz: float, dt: float):
        rc = 1.0 / (2.0 * np.pi * cutoff_hz)
        self.alpha = rc / (rc + dt)
        self._prev = None

    def __call__(self, x: float) -> float:
        if self._prev is None:
            self._prev = x
        self._prev = self.alpha * self._prev + (1.0 - self.alpha) * x
        return self._prev

    def reset(self):
        self._prev = None


class DifferentiatorWithFilter:
    """Filtered numerical derivative for converting encoder position to velocity.

    Computes dx/dt with a first-order low-pass filter to reject quantization noise.

    Args:
        cutoff_hz: Low-pass cutoff [Hz]. Typical: 50-100 Hz.
        dt: Sample period [s].
    """

    def __init__(self, cutoff_hz: float = 50.0, dt: float = 0.002):
        self.dt = dt
        self._lpf = LowPassFilter(cutoff_hz, dt)
        self._prev_x = None

    def __call__(self, x: float) -> float:
        if self._prev_x is None:
            self._prev_x = x
            return 0.0

        raw_deriv = (x - self._prev_x) / self.dt
        self._prev_x = x
        return self._lpf(raw_deriv)

    def reset(self):
        self._prev_x = None
        self._lpf.reset()
