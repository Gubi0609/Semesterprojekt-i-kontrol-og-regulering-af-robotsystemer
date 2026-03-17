"""Simple CSV data logger for experiment recording."""

import csv
import time
from pathlib import Path


class DataLogger:
    """Logs timestamped state + control data to CSV.

    Usage:
        logger = DataLogger("experiment_01")
        logger.log(t, state, voltage)
        ...
        logger.close()
    """

    def __init__(self, name: str = "run", output_dir: str = "logs"):
        self._dir = Path(output_dir)
        self._dir.mkdir(parents=True, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self._path = self._dir / f"{name}_{timestamp}.csv"
        self._file = open(self._path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(["t", "theta", "alpha", "theta_dot", "alpha_dot", "voltage"])
        self._count = 0

    def log(self, t: float, state, voltage: float):
        self._writer.writerow([
            f"{t:.6f}",
            f"{state.theta:.6f}",
            f"{state.alpha:.6f}",
            f"{state.theta_dot:.6f}",
            f"{state.alpha_dot:.6f}",
            f"{voltage:.6f}",
        ])
        self._count += 1

    def close(self):
        self._file.close()
        print(f"Logged {self._count} samples to {self._path}")

    @property
    def filepath(self) -> Path:
        return self._path
