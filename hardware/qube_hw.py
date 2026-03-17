"""Hardware interface to the real Qube-Servo 3 via Quanser HIL API.

Requires the Quanser SDK to be installed:
    Linux:   https://github.com/quanser/quanser_sdk_linux
    Windows: https://github.com/quanser/quanser_sdk_win64

Install Python bindings:
    sudo apt install python3-quanser-apis   (Ubuntu)

Run calibrate.py once before first use to determine encoder offsets.
"""

import json
import numpy as np
from pathlib import Path
from interface import QubeInterface, QubeState
from utils.filters import DifferentiatorWithFilter

try:
    from quanser.hardware import HIL, Clock
    _HIL_AVAILABLE = True
except ImportError:
    _HIL_AVAILABLE = False


# Qube-Servo 3 channel mappings (from QUARC docs)
_ENCODER_CHANNELS = np.array([0, 1], dtype=np.uint32)  # 0=motor, 1=pendulum
_ANALOG_CHANNELS = np.array([0], dtype=np.uint32)       # motor voltage
_DIGITAL_CHANNELS = np.array([0], dtype=np.uint32)       # motor enable

# Encoder resolution: 512 lines × 4X quadrature = 2048 counts/rev
# (Confirmed by Qube-Servo 3 datasheet: "2,048 counts/revolution" in quadrature mode)
_DEFAULT_COUNTS_PER_REV = 2048


class HardwareQube(QubeInterface):
    """Interface to the physical Qube-Servo 3.

    Loads calibration from calibration.json if available. Run calibrate.py
    first to generate this file.

    Args:
        dt: Control loop period [s]. Default 2 ms (500 Hz).
        board_id: Board identifier string. "0" for first connected unit.
        velocity_filter_hz: Cutoff freq for encoder-derived velocity [Hz].
        calibration_path: Path to calibration.json.
    """

    def __init__(
        self,
        dt: float = 0.002,
        board_id: str = "0",
        velocity_filter_hz: float = 50.0,
        calibration_path: str = "calibration.json",
    ):
        if not _HIL_AVAILABLE:
            raise RuntimeError(
                "Quanser HIL Python API not found. Install the Quanser SDK:\n"
                "  https://github.com/quanser/quanser_sdk_linux\n"
                "  sudo apt install python3-quanser-apis"
            )

        self._dt = dt
        self._card = HIL("qube_servo3_usb", board_id)

        # Load calibration
        self._cal = self._load_calibration(calibration_path)
        self._counts_per_rev = self._cal.get("counts_per_rev", _DEFAULT_COUNTS_PER_REV)
        self._pend_offset = self._cal.get("pendulum_offset_counts", 0)
        self._pend_sign = self._cal.get("pendulum_sign_flip", 1)
        self._counts_to_rad = (2.0 * np.pi) / self._counts_per_rev

        # Velocity estimators (encoders give position only)
        self._theta_diff = DifferentiatorWithFilter(velocity_filter_hz, dt)
        self._alpha_diff = DifferentiatorWithFilter(velocity_filter_hz, dt)

        # Buffers
        self._enc_buffer = np.zeros(2, dtype=np.int32)
        self._analog_buffer = np.zeros(1, dtype=np.float64)

        # Enable the motor amplifier
        self._card.write_digital(_DIGITAL_CHANNELS, 1, np.array([1], dtype=np.int8))

    @staticmethod
    def _load_calibration(path: str) -> dict:
        p = Path(path)
        if p.exists():
            with open(p) as f:
                cal = json.load(f)
            print(f"[hardware] Loaded calibration from {p}")
            print(f"  offset={cal.get('pendulum_offset_counts', 0)} counts, "
                  f"sign_flip={cal.get('pendulum_sign_flip', 1)}")
            return cal
        else:
            print(f"[hardware] No calibration file found at {p}")
            print(f"  Using defaults (run calibrate.py for accurate results)")
            return {}

    @property
    def dt(self) -> float:
        return self._dt

    def reset(self) -> QubeState:
        """Zero motor encoder and reset filters.

        NOTE: Does NOT zero the pendulum encoder — that would lose
        the calibration. The offset is applied in software.
        """
        # Zero only the motor encoder
        self._card.set_encoder_counts(
            np.array([0], dtype=np.uint32), 1,
            np.array([0], dtype=np.int32),
        )
        self._theta_diff.reset()
        self._alpha_diff.reset()

        # Write zero voltage
        self._card.write_analog(_ANALOG_CHANNELS, 1, np.array([0.0], dtype=np.float64))

        return self._read_state()

    def step(self, voltage: float) -> QubeState:
        """Write voltage, read encoders, return state."""
        voltage = float(np.clip(voltage, -10.0, 10.0))

        # Write motor voltage
        self._analog_buffer[0] = voltage
        self._card.write_analog(_ANALOG_CHANNELS, 1, self._analog_buffer)

        # Read encoders
        self._card.read_encoder(_ENCODER_CHANNELS, 2, self._enc_buffer)

        return self._read_state()

    def _read_state(self) -> QubeState:
        """Convert raw encoder counts to calibrated state."""
        self._card.read_encoder(_ENCODER_CHANNELS, 2, self._enc_buffer)

        theta = self._enc_buffer[0] * self._counts_to_rad

        # Apply pendulum calibration: offset + sign flip
        # Result: alpha=0 is upright, alpha=±pi is hanging down
        alpha_raw = self._enc_buffer[1]
        alpha = (alpha_raw - self._pend_offset) * self._pend_sign * self._counts_to_rad

        theta_dot = self._theta_diff(theta)
        alpha_dot = self._alpha_diff(alpha) * self._pend_sign

        return QubeState(
            theta=theta,
            alpha=alpha,
            theta_dot=theta_dot,
            alpha_dot=alpha_dot,
        )

    def close(self) -> None:
        """Safe shutdown: zero voltage, disable amp, close card."""
        try:
            self._card.write_analog(
                _ANALOG_CHANNELS, 1, np.array([0.0], dtype=np.float64)
            )
            self._card.write_digital(
                _DIGITAL_CHANNELS, 1, np.array([0], dtype=np.int8)
            )
        finally:
            self._card.close()
