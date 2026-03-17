"""Visualization for the rotary inverted pendulum.

Supports three output modes (auto-detected, or forced via constructor):
  1. "browser"  — interactive Plotly HTML, opens in default browser (default)
  2. "gui"      — matplotlib TkAgg window
  3. "png"      — saves static PNG to disk (headless fallback)
"""

import numpy as np
from collections import deque
from pathlib import Path
import time as _time


class PendulumVisualizer:
    """Records and visualizes a simulation run.

    Usage:
        viz = PendulumVisualizer()
        for ...:
            viz.update(state, voltage, t)
        viz.show()

    Args:
        history_len: Max data points to retain.
        mode: "browser" (plotly), "gui" (matplotlib), "png", or "auto".
        save_dir: Output directory for png/html files.
    """

    def __init__(
        self,
        history_len: int = 50000,
        mode: str = "auto",
        save_dir: str = "plots",
    ):
        self._history_len = history_len
        self._save_dir = Path(save_dir)
        self._mode = self._resolve_mode(mode)

        self._t: deque[float] = deque(maxlen=history_len)
        self._theta: deque[float] = deque(maxlen=history_len)
        self._alpha: deque[float] = deque(maxlen=history_len)
        self._voltage: deque[float] = deque(maxlen=history_len)

        print(f"[visualizer] mode={self._mode}")

    @staticmethod
    def _resolve_mode(mode: str) -> str:
        if mode != "auto":
            return mode
        # Prefer browser (plotly) — works everywhere with zero display config
        try:
            import plotly
            return "browser"
        except ImportError:
            pass
        return "png"

    def update(self, state, voltage: float, t: float):
        """Record one data point."""
        self._t.append(t)
        self._theta.append(np.degrees(state.theta))
        self._alpha.append(np.degrees(state.alpha))
        self._voltage.append(voltage)

    def show(self):
        """Render the recorded data."""
        if self._mode == "browser":
            self._show_plotly()
        elif self._mode == "gui":
            self._show_matplotlib()
        else:
            self._show_png()

    # ---- Plotly (browser) ------------------------------------------------

    def _show_plotly(self):
        from plotly.subplots import make_subplots
        import plotly.graph_objects as go

        t = list(self._t)
        alpha = list(self._alpha)
        theta = list(self._theta)
        volts = list(self._voltage)

        fig = make_subplots(
            rows=3, cols=1,
            shared_xaxes=True,
            vertical_spacing=0.06,
            subplot_titles=(
                "Pendulum Angle α (0° = upright)",
                "Arm Angle θ",
                "Motor Voltage",
            ),
        )

        fig.add_trace(
            go.Scattergl(x=t, y=alpha, mode="lines", name="α [deg]",
                         line=dict(color="red", width=1)),
            row=1, col=1,
        )
        # Reference line at 0
        fig.add_hline(y=0, line_dash="dash", line_color="gray",
                      opacity=0.5, row=1, col=1)

        fig.add_trace(
            go.Scattergl(x=t, y=theta, mode="lines", name="θ [deg]",
                         line=dict(color="blue", width=1)),
            row=2, col=1,
        )
        fig.add_hline(y=0, line_dash="dash", line_color="gray",
                      opacity=0.5, row=2, col=1)

        fig.add_trace(
            go.Scattergl(x=t, y=volts, mode="lines", name="V [V]",
                         line=dict(color="black", width=1)),
            row=3, col=1,
        )
        fig.add_hline(y=10, line_dash="dot", line_color="red",
                      opacity=0.3, row=3, col=1)
        fig.add_hline(y=-10, line_dash="dot", line_color="red",
                      opacity=0.3, row=3, col=1)

        fig.update_yaxes(title_text="α [deg]", row=1, col=1)
        fig.update_yaxes(title_text="θ [deg]", row=2, col=1)
        fig.update_yaxes(title_text="V [V]", row=3, col=1)
        fig.update_xaxes(title_text="Time [s]", row=3, col=1)

        fig.update_layout(
            title="Qube-Servo 3 — Inverted Pendulum Simulation",
            height=800,
            showlegend=False,
            template="plotly_white",
        )

        # Save and open
        self._save_dir.mkdir(parents=True, exist_ok=True)
        ts = _time.strftime("%Y%m%d_%H%M%S")
        path = self._save_dir / f"sim_result_{ts}.html"
        fig.write_html(str(path), auto_open=True)
        print(f"[visualizer] Interactive plot opened in browser: {path}")

    # ---- Matplotlib GUI --------------------------------------------------

    def _show_matplotlib(self):
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt

        fig, axes = self._make_mpl_figure(plt)
        plt.ioff()
        plt.show()

    # ---- Matplotlib PNG --------------------------------------------------

    def _show_png(self):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = self._make_mpl_figure(plt)

        self._save_dir.mkdir(parents=True, exist_ok=True)
        ts = _time.strftime("%Y%m%d_%H%M%S")
        path = self._save_dir / f"sim_result_{ts}.png"
        fig.savefig(path, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"[visualizer] Plot saved to {path}")

    def _make_mpl_figure(self, plt):
        t = list(self._t)
        alpha = list(self._alpha)
        theta = list(self._theta)
        volts = list(self._voltage)

        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 7), sharex=True)

        ax1.plot(t, alpha, "r-", lw=0.8)
        ax1.axhline(0, color="gray", ls="--", alpha=0.5)
        ax1.set_ylabel("α [deg]")
        ax1.set_title("Pendulum Angle (0° = upright)")
        ax1.grid(True, alpha=0.3)

        ax2.plot(t, theta, "b-", lw=0.8)
        ax2.axhline(0, color="gray", ls="--", alpha=0.5)
        ax2.set_ylabel("θ [deg]")
        ax2.set_title("Arm Angle")
        ax2.grid(True, alpha=0.3)

        ax3.plot(t, volts, "k-", lw=0.8)
        ax3.axhline(10, color="red", ls=":", alpha=0.3)
        ax3.axhline(-10, color="red", ls=":", alpha=0.3)
        ax3.set_ylabel("V [V]")
        ax3.set_xlabel("Time [s]")
        ax3.set_title("Motor Voltage")
        ax3.grid(True, alpha=0.3)

        fig.suptitle("Qube-Servo 3 — Inverted Pendulum Simulation", fontsize=13)
        fig.tight_layout()
        return fig, (ax1, ax2, ax3)
